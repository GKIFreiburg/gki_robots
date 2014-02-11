#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <kobuki_msgs/SensorState.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
bool data_available = false;

// EStop Safety Controller
// Stop the robot if estop doesn't signal GO
// Be silent otherwise.

void stop_robot()
{
    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = 0;
    cmd_msg.linear.y = 0;
    cmd_msg.linear.z = 0;
    cmd_msg.angular.x = 0;
    cmd_msg.angular.y = 0;
    cmd_msg.angular.z = 0;
    pub.publish(cmd_msg);
}

void estop(const std_msgs::Bool & input)
{
    if(!input.data) {
        stop_robot();
        ROS_WARN_THROTTLE(5.0, "Stopping Robot due to EStop pressed.");
    }

    data_available = true;
}

void kobuki_core(const kobuki_msgs::SensorState & sensors)
{
    bool din3_on = (sensors.digital_input & kobuki_msgs::SensorState::DIGITAL_INPUT3)
        == kobuki_msgs::SensorState::DIGITAL_INPUT3;
    if(!din3_on) {  // EStop not on GO
        stop_robot();
        ROS_WARN_THROTTLE(5.0, "Stopping Robot due to EStop pressed.");
    }

    data_available = true;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "estop_safety_controller");
    ros::NodeHandle nh;

    ros::NodeHandle nhPriv("~");

    bool auto_estop = true;
    nh.param("auto_estop", auto_estop, auto_estop);
    bool use_kobuki_din3_estop = false;
    nh.param("use_kobuki_din3_estop", use_kobuki_din3_estop, use_kobuki_din3_estop);

    ros::Subscriber sub = nh.subscribe ("estop", 1, estop);
    if(use_kobuki_din3_estop)
        ros::Subscriber subKobuki = nh.subscribe("mobile_base/sensors/core", 1, kobuki_core);
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/estop", 1);

    ros::Rate r(1);   //    1Hz
    while (ros::ok()) {
        if(!data_available) {
            if(auto_estop) {
                ROS_WARN_THROTTLE(1.0, "No EStop data available - Stopping robot!");
                stop_robot();
            } else {
                ROS_WARN_THROTTLE(5.0, "No EStop data available!");
            }
        }

        data_available = false;

        ros::spinOnce();
        r.sleep();
    }
}

