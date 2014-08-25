#include <ros/ros.h>
#include "axis_camera/Axis.h"
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>

ros::Publisher pubJointStates;

void axis_callback(const axis_camera::Axis & msg)
{
    sensor_msgs::JointState js;
    js.name.push_back("axis_pan");
    js.name.push_back("axis_tilt");
    js.position.resize(2);

    js.position[0] = angles::from_degrees(msg.pan);
    js.position[1] = angles::from_degrees(msg.tilt);

    pubJointStates.publish(js);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "axis_to_joint_states");
    ros::NodeHandle nh;

    ros::Subscriber subAxis = nh.subscribe("state", 5, axis_callback);
    pubJointStates = nh.advertise<sensor_msgs::JointState>("/joint_states", 5);

    ros::spin();

    return 0;
}
