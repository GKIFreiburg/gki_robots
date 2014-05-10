#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>

ros::Publisher pub;
ros::Time lastDataTime(0);

int stop_button = 0;
int go_button = 1;
bool estop_on_go = true;

// Joystick EStop
// Send Estop if the stop_button was pressed until
// the go_button is pressed again.

void send_estop()
{
    std_msgs::Bool estop;
    estop.data = estop_on_go;
    pub.publish(estop);
}

void joy_callback(const sensor_msgs::Joy & joy)
{
    bool estop_on_go_old = estop_on_go;
    if(joy.buttons[go_button])
        estop_on_go = true;
    // Do stop later -> If both pressed, we are in stop
    if(joy.buttons[stop_button])
        estop_on_go = false;

    if(estop_on_go_old != estop_on_go) {
        ROS_INFO("Changed joystick_estop to %s", estop_on_go ? "GO" : "STOP");
    }

    lastDataTime = ros::Time::now();
    send_estop();
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "joystick_estop");
    ros::NodeHandle nh;

    ros::NodeHandle nhPriv("~");

    bool auto_estop = true;
    nhPriv.param("auto_estop", auto_estop, auto_estop);
    nhPriv.param("stop_button", stop_button, stop_button);
    nhPriv.param("go_button", go_button, go_button);
    if(stop_button == go_button) {
        ROS_FATAL("stop_button (%d) == go_button (%d) - This cannot work.", stop_button, go_button);
        return 1;
    }

    ros::Subscriber sub = nh.subscribe ("joy", 1, joy_callback);
    pub = nh.advertise<std_msgs::Bool>("joystick_estop", 1);

    ros::Rate r(10);
    while (ros::ok()) {
        ros::Duration timeSinceLastData = ros::Time::now() - lastDataTime;
        if(timeSinceLastData > ros::Duration(5.0)) {
            if(auto_estop) {
                ROS_WARN_THROTTLE(1.0, "No EStop data available - Stopping robot!");
                estop_on_go = false;
            } else {
                ROS_WARN_THROTTLE(5.0, "No EStop data available!");
            }
        }

        ros::spinOnce();
        send_estop();
        r.sleep();
    }
}

