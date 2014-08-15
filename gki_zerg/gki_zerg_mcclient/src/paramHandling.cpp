#include "paramHandling.h"

#include <iostream>
#include <string>
#include "gki_utils/misc.h"
#include "gki_utils/parameters_ext.h"
using namespace std;

namespace McClientParams {
//int g_Estop = 1;

//double g_3dScanStartAngle = -20.0;
//double g_3dScanEndAngle = 20.0;
//double g_3dScanAngleStep = 0.5;
//double g_KeepLaserPitch = 0.0;

int g_OdometryUseMagneticYaw = 0;

void subscribeParams(bool setIfNotOnParamServer)
{
    static ros::NodeHandle* nh = new ros::NodeHandle("~");

    if(setIfNotOnParamServer) {
        if(!nh->hasParam("odometry_use_magnetic_yaw"))
            nh->setParam("odometry_use_magnetic_yaw", g_OdometryUseMagneticYaw);
    }

    parameters_ext::subscribe(nh,
            "odometry_use_magnetic_yaw", &g_OdometryUseMagneticYaw);

    printf("params:\n");
    //printf("\tESTOP:                  \t%d\n", g_Estop);
    //printf("\t3d scan start angle:     \t%.2f\n", g_3dScanStartAngle);
    //printf("\t3d scan end angle:     \t%.2f\n",   g_3dScanEndAngle);
    //printf("\t3d scan angle step:     \t%.2f\n",  g_3dScanAngleStep);
    //printf("\tKeep laser pitch:     \t%.2f\n",  g_KeepLaserPitch);
    printf("\tOdometry use magnetic yaw: \t%d\n", g_OdometryUseMagneticYaw);
}

}

