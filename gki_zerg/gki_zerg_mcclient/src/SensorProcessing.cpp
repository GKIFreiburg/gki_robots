#include <math.h>
#include "SensorProcessing.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "angles/angles.h"
#include "ConfigParser.h"
#include <nav_msgs/Odometry.h>
#include <iostream>
#include "slip.h"
#include <tf/tf.h>
#include "paramHandling.h"

using std::cout;

extern bool g_Odo2;

double last_front = 0, last_last_front = 0, last_rear = 0, last_last_rear = 0;

// Defines the value from which we consider
// the touch return as a touchdown
SensorProcessing::SensorProcessing(ConfigParser* config, bool processOdo, double odoX, double odoY, double odoTh, bool storeOdo)
{
    this->config = config;

    stall = 0;

    _processOdometry = processOdo;
    _storeOdometry = storeOdo;

    odometry_pos_x = odoX;
    odometry_pos_y = odoY;
    odometry_pos_th = odoTh;

    // From the values that are integrated.
    odometry_last_dist = 0;
    odometry_vel_trans = 0;
    odometry_vel_rot = 0;
    last_cube_heading = 0;

    odometry_left_front_dx = 0;
    odometry_left_front_vel = 0;
    odometry_left_rear_dx = 0;
    odometry_left_rear_vel = 0;
    odometry_right_front_dx = 0;
    odometry_right_front_vel = 0;
    odometry_right_rear_dx = 0;
    odometry_right_rear_vel = 0;
    rotation_vel = 0;
    cycle_time = 0;

    infrared_distance = new unsigned short[config->numSensorsIR()];
    ultra_sonic_distance = new unsigned short[config->numSensorsUS()];

    flex_value = new unsigned short[config->numSensorsFlex()];
    bumper_value = new unsigned short[config->numSensorsBumper()];
    tpa81_values = new unsigned short[config->numSensorsTPA81() *
    TPA_DATA_SIZE];

    pyro_last_angle = 0;
    pyro_last_value = 0;
    pyro_last_time = 0;
    pyro_lowpassed_value = 0;
    pyro_maxdiff_value = 86.8742;
    pyro_mindiff_value = 22.8526;

    pyro_var_min_value = 1600.0;
    pyro_var_max_value = 10000.0;

    co2_temperature = 0;
    co2_concentration = 0;

    ros::NodeHandle nh;
    pubOdom = nh.advertise<nav_msgs::Odometry>("odom", 1);
    diagnostics = new diagnostic_updater::Updater();
    diagnostics->setHardwareID("Zerg");
    minFreq = 30;
    maxFreq = 30;
    odomDiagnosticPublisher = new diagnostic_updater::DiagnosedPublisher<nav_msgs::Odometry>(pubOdom, *diagnostics, diagnostic_updater::FrequencyStatusParam(&minFreq, &maxFreq, 0.2, 10), diagnostic_updater::TimeStampStatusParam(0.0, 0.015));

}

SensorProcessing::~SensorProcessing()
{

}

void SensorProcessing::setProcessOdo(bool p)
{
    _processOdometry = p;
}

void SensorProcessing::filterOdometry()
{
    if (!_processOdometry)
        return;
    double mean_velocity = ((double) odometry_left_front_vel + (double) odometry_right_front_vel + double(odometry_left_rear_vel) + double(odometry_right_rear_vel)) / 4.0;

    double mean_dist = ((double) odometry_left_front_dx + (double) odometry_right_front_dx + double(odometry_left_rear_dx) + double(odometry_right_rear_dx)) / 4.0;

    odometry_vel_trans = (int) (((float) config->getWheelOutline() / (float) WHEEL_TICKS_PER_CIRCULATION) * ((float) mean_velocity));
    double ticks_per_90deg_per_sec = 19967.0;
    double ticks_per_deg_per_sec = ticks_per_90deg_per_sec/90.0;
    odometry_vel_rot = rotation_vel / ticks_per_deg_per_sec;
    // get odometry_vel_rot to deg/s
    //printf("odometry_vel_rot: %f\n", odometry_vel_rot);

    if(McClientParams::g_OdometryEstimateThetaWithoutImu) {
        // either estimate from rotation_vel
        // or from delta of velocities/dists
        static ros::WallTime lastUpdate = ros::WallTime::now();
        ros::WallTime now = ros::WallTime::now();
        double dt = (now - lastUpdate).toSec();
        lastUpdate = now;
        odometry_pos_th += odometry_vel_rot * dt;
    }

    double new_dist = ((double) config->getWheelOutline() / (double) WHEEL_TICKS_PER_CIRCULATION) * ((double) mean_dist);

    double delta_dist = new_dist - odometry_last_dist;

    odometry_last_dist = new_dist;

    if (stall == 0)
    {
        odometry_pos_x += cos(angles::from_degrees((double) getOdometryPosTh())) * delta_dist;
        odometry_pos_y += sin(angles::from_degrees((double) getOdometryPosTh())) * delta_dist;
    }

    // store odo on param server periodically
    if(_storeOdometry) {
        static ros::Time lastOdoStoreTime = ros::Time(0);
        if(ros::Time::now() - lastOdoStoreTime > ros::Duration(1.0)) {
            static ros::NodeHandle nhPriv("~");
            nhPriv.setParam("odometry_x", odometry_pos_x);
            nhPriv.setParam("odometry_y", odometry_pos_y);
            nhPriv.setParam("odometry_th", odometry_pos_th);
            lastOdoStoreTime = ros::Time::now();
        }
    }
}

void SensorProcessing::update()
{
    diagnostics->update();
}

void SensorProcessing::updateTime(timeval* time)
{
    if (time == NULL)
        gettimeofday(&currenttime, NULL);
    else
        currenttime = *time;

    if (starttime == 0)
    {
        starttime = currenttime.tv_sec * 1000 + ((currenttime.tv_usec / 1000) % 1000);
    }
}

long SensorProcessing::getRelativeTime_ms()
{
    return (currenttime.tv_sec * 1000 + ((currenttime.tv_usec / 1000) % 1000)) - starttime;
}

double SensorProcessing::getOdometryPosX()
{
    return odometry_pos_x;
}

double SensorProcessing::getOdometryPosZ()
{
    return odometry_pos_y;
}

double SensorProcessing::getOdometryPosTh()
{
    if(McClientParams::g_OdometryUseImu)
        return last_cube_heading;
    else
        return odometry_pos_th;
}

int SensorProcessing::getOdometryTransVel()
{
    return odometry_vel_trans;
}

int SensorProcessing::getOdometryRotVel()
{
    return odometry_vel_rot;
}

void SensorProcessing::publishSensorData()
{
    if (config->enabledStartButton())
    {
        // --CHANGES:BEGIN--
        // msg name changed
        // from rescue_odometry_message to carmen_base_odometry_message
        nav_msgs::Odometry msg;
        TransValues tValues;

        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_footprint";
        msg.header.stamp = ros::Time::now();
        //geometry_msgs/PoseWithCovariance pose
        //geometry_msgs/TwistWithCovariance twist

        // --CHANGES:BEGIN--
        // unit changed
        // from mm to m
        msg.pose.pose.position.x = odometry_pos_x / 1000.0;
        msg.pose.pose.position.y = odometry_pos_y / 1000.0;
        // --CHANGES:END--

        // --CHANGES:BEGIN--
        // unit changed
        // from deg to rad
        double theta = angles::from_degrees(getOdometryPosTh());
        msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        // --CHANGES:END--

        //std::cerr << "(x,y,th) (" <<  msg.x << "," << msg.y <<
//	"," << getOdometryPosTh() << ")" << std::endl;

        // --CHANGES:BEGIN--
        // new variable acceleration
        // TODO: calculate it !!
        msg.twist.twist.linear.x = odometry_vel_trans / 1000.0;
        msg.twist.twist.angular.z = angles::from_degrees(odometry_vel_rot); // TODO
        // --CHANGES:END--

        // --CHANGES:BEGIN--
        // new struct with tranlation values
        tValues.transVelLeftFront = (int) (odometry_left_front_vel * ((float) config->getWheelOutline() / (float) WHEEL_TICKS_PER_CIRCULATION));

        tValues.transVelLeftRear = (int) (odometry_left_rear_vel * ((float) config->getWheelOutline() / (float) WHEEL_TICKS_PER_CIRCULATION));

        tValues.transVelRightFront = (int) (odometry_right_front_vel * ((float) config->getWheelOutline() / (float) WHEEL_TICKS_PER_CIRCULATION));

        tValues.transVelRightRear = (int) (odometry_right_rear_vel * ((float) config->getWheelOutline() / (float) WHEEL_TICKS_PER_CIRCULATION));
        // --CHANGES:END--
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = msg.header.stamp;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = msg.pose.pose.position.x;
        odom_trans.transform.translation.y = msg.pose.pose.position.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = msg.pose.pose.orientation;

        odom_broadcaster.sendTransform(odom_trans);
        if (config->enabledOdometrylf() || config->enabledOdometrylr() || config->enabledCompass2() || config->enabledOdometryrf() || config->enabledOdometryrr() || config->enabledOdometryrot())
        {
            detectSlip(tValues);
            odomDiagnosticPublisher->publish(msg);
            //pubOdom.publish(msg);
        }
    }
}
