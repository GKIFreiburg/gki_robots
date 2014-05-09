#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <kobuki_msgs/SensorState.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;
ros::Time lastDataTime(0);

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
        ROS_WARN_THROTTLE(5.0, "Stopping Robot due to EStop pressed (via msg).");
    }

    lastDataTime = ros::Time::now();
}

void kobuki_core(const kobuki_msgs::SensorState & sensors)
{
    bool estop_pressed = (sensors.digital_input & kobuki_msgs::SensorState::DIGITAL_INPUT3)
        == kobuki_msgs::SensorState::DIGITAL_INPUT3;
    if(estop_pressed) {
        stop_robot();
        ROS_WARN_THROTTLE(5.0, "Stopping Robot due to EStop pressed.");
    }

    lastDataTime = ros::Time::now();
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "estop_safety_controller");
    ros::NodeHandle nh;

    ros::NodeHandle nhPriv("~");

    bool auto_estop = true;
    nhPriv.param("auto_estop", auto_estop, auto_estop);
    bool use_kobuki_din3_estop = false;
    nhPriv.param("use_kobuki_din3_estop", use_kobuki_din3_estop, use_kobuki_din3_estop);

    ros::Subscriber sub = nh.subscribe ("estop", 1, estop);
    ros::Subscriber subKobuki;
    if(use_kobuki_din3_estop)
        subKobuki = nh.subscribe("mobile_base/sensors/core", 1, kobuki_core);
    pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/estop", 1);

    ROS_INFO("estop_safety_controller: auto_estop: %s use_kobuki_din3_estop: %s",
            auto_estop ? "enabled" : "disabled", use_kobuki_din3_estop ? "enabled" : "disabled");

    ros::Rate r(10);
    while (ros::ok()) {
        ros::Duration timeSinceLastData = ros::Time::now() - lastDataTime;
        if(timeSinceLastData > ros::Duration(1.0)) {
            if(auto_estop) {
                ROS_WARN_THROTTLE(1.0, "No EStop data available - Stopping robot!");
                stop_robot();
            } else {
                ROS_WARN_THROTTLE(5.0, "No EStop data available!");
            }
        }

        ros::spinOnce();
        r.sleep();
    }
}

