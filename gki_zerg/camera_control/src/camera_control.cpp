#include "camera_control/camera_control.h"

 CameraControl::CameraControl(std::string name) :
    as_(nh_, name, boost::bind(&CameraControl::executeCB, this, _1), false),action_name_(name){
    camera_pan_offset_ = angles::from_degrees(90);  
    camera_sweep_start_ = angles::from_degrees(75);
    camera_sweep_end_ =  angles::from_degrees(100 - 360);
    camera_command_.pan = 10.0;
    camera_command_.tilt = 10.0;
    camera_command_.zoom = 0.0;
    camera_command_.focus = 0.0;
    camera_command_.brightness = 0.0;
    camera_command_.iris = 0.0;
    camera_command_.autofocus = true;
    axis_camera_pub_ =  nh_.advertise<axis_camera::Axis>("cmd", 1);
    as_.start();
}

  CameraControl::~CameraControl(void){
      //todo
  }

  ///based on hte goal implement two function
  ///fist sweep scan function
  ///adjuct pan and tilt 
  ///here all the angles and value must be based on the robot coordinate system
  ///find the right transformation for the pan and tilt value
  ///values need to be in radian and second 
  ///after execution each command checlk the this topic /axis/state/ and 
  ///wait until you have reach the goal and then send another command 
  ///check the invalid and out of range values before executing
  ///

  bool CameraControl::SetCameraAngels(double _pan, double _tilt){
    camera_command_.pan = -angles::to_degrees(_pan + camera_pan_offset_);
    camera_command_.tilt = -angles::to_degrees(_tilt);
   
    axis_camera_pub_.publish(camera_command_);
    return true;
  }

  bool CameraControl::DoSweepScan(const camera_control_msgs::CameraGoalConstPtr &goal){
   ros::Rate r(1/goal->sweep_hold_time);
   for(double i = camera_sweep_start_; i > camera_sweep_end_; i -= goal->sweep_step){
       SetCameraAngels(i,goal->tilt); 
       r.sleep(); 
   }
   return true;
  }

  void CameraControl::executeCB(const camera_control_msgs::CameraGoalConstPtr &goal){
    // helper variables
    ros::Rate r(1);
    bool success = true;
    

    // publish info to the console for the user
    ROS_INFO("%s: Executing  ", action_name_.c_str());

    // start executing the action
      // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok()){
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
    }

    if(goal->command == camera_control_msgs::CameraGoal::SWEEP){
          DoSweepScan(goal);
    }else if(goal->command == camera_control_msgs::CameraGoal::FIXEDPOS){
          SetCameraAngels(goal->pan, goal->tilt);
    }else{
          success = false;
    }

    if(success){
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }else{
       as_.setAborted();
    }
  }


