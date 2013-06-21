#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <turtlebot_button/Buttons.h>
#include <geometry_msgs/Twist.h>


ros::Publisher pub;


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
}

//fraglich for no data
void etstop()
{

}

int main (int argc, char **argv){
    //Initialize ROS
    ros::init(argc, argv, "box_dector");
    ros::NodeHandle nh;


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/estop", 1, estop);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux", 1);


    // Spin
    ros::spin ();
}
