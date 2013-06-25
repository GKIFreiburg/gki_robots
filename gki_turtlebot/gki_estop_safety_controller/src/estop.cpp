#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <turtlebot_button/Buttons.h>
#include <geometry_msgs/Twist.h>


ros::Publisher pub;
bool data_available = false;


void estop(const std_msgs::Bool input)
{
    if (!input.data)
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

    data_available = true;
}

int main (int argc, char **argv){
    //Initialize ROS
    ros::init(argc, argv, "estop_safety_controller");
    ros::NodeHandle nh;

    //edit path!!!
    int safety_level = 1;
    //nh.getParam("gki_estop_safety_controller/../safety_level",safety_level);

    // Create a ROS subscriber for estop topic
    ros::Subscriber sub = nh.subscribe ("/estop", 1, estop);

    // Create a ROS publisher for the output estop
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/estop", 1);


    //check with 1Hz
    ros::Rate r(1);   //1Hz

    while (1)
    {
        //switch over safety level
        switch(safety_level)
        {
        case 0:
            if(!data_available)
            {
                ROS_WARN_STREAM("STOPPING WHILE THERE WAS NO ESTOP DATA!");
                geometry_msgs::Twist cmd_msg;
                cmd_msg.linear.x = 0;
                cmd_msg.linear.y = 0;
                cmd_msg.linear.z = 0;
                cmd_msg.angular.x = 0;
                cmd_msg.angular.y = 0;
                cmd_msg.angular.z = 0;
                pub.publish(cmd_msg);
            }
            break;
        case 1:
            if(!data_available)
            {
                ROS_WARN_STREAM("NO ESTOP DATA!");
            }
            break;
        }

        //reset value for available data
        data_available = false;

        ros::spinOnce();
        r.sleep();
    }

    // Spin
    ros::spin ();
}
