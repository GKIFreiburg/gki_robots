#include <ros/ros.h>
#include <camera_control/camera_control.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "axis_camera");
  CameraControl cameracontrol(ros::this_node::getName());
  ros::spin();

  return 0;
}
