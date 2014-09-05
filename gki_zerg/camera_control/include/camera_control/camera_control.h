#ifndef CAMERACONTROL_H
#define CAMERACONTROL_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <camera_control_msgs/CameraAction.h>
#include <axis_camera/Axis.h>
#include <angles/angles.h>


class CameraControl{

    protected:

              ros::NodeHandle nh_;
              // NodeHandle instance must be created before this line. Otherwise strange error may occur.
              actionlib::SimpleActionServer<camera_control_msgs::CameraAction> as_; 
              std::string action_name_;
              // create messages that are used to published feedback/result
              camera_control_msgs::CameraFeedback feedback_;
              camera_control_msgs::CameraResult result_;
              ros::Publisher axis_camera_pub_;
              axis_camera::Axis camera_command_;
              double camera_pan_offset_;
              double camera_sweep_start_;
              double camera_sweep_end_;

    public:
   
              void executeCB(const camera_control_msgs::CameraGoalConstPtr &goal);
              bool DoSweepScan(const camera_control_msgs::CameraGoalConstPtr &goal);
              bool SetCameraAngels(double _pan, double _tilt);
              CameraControl(std::string name);
              ~CameraControl();
 };


#endif
