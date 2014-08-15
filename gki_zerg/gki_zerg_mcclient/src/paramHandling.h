#ifndef MC_CLIENT_PARAM_HANDLING_H
#define MC_CLIENT_PARAM_HANDLING_H

#include <ros/ros.h>

namespace McClientParams {
   //extern int g_Estop;

   //extern double g_3dScanStartAngle;
   //extern double g_3dScanEndAngle;
   //extern double g_3dScanAngleStep;
   //extern double g_KeepLaserPitch;

   extern int g_OdometryUseMagneticYaw;

   void subscribeParams(bool setIfNotOnParamServer);
}

#endif

