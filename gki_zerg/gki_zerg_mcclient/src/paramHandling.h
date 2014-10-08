#ifndef MC_CLIENT_PARAM_HANDLING_H
#define MC_CLIENT_PARAM_HANDLING_H

#include <ros/ros.h>

namespace McClientParams {
   //extern int g_Estop;

   //extern double g_3dScanStartAngle;
   //extern double g_3dScanEndAngle;
   //extern double g_3dScanAngleStep;
   //extern double g_KeepLaserPitch;

    /// get theta from imu
    extern bool g_OdometryUseImu;
    /// only relevant when g_OdometryUseImu is false.
    /// Should we try to estimate some theta from vels or keep it at zero
    extern bool g_OdometryEstimateThetaWithoutImu;

   extern int g_OdometryUseMagneticYaw;

   void subscribeParams(bool setIfNotOnParamServer);
}

#endif

