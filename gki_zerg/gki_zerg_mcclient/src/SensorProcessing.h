#ifndef __SENSORPROCESSING_H__
#define __SENSORPROCESSING_H__

//#include <qmutex.h>
#include <tf/transform_broadcaster.h>
#include <ros/publisher.h>
//#include <ros/ros.h>
#include <stddef.h>
#include <sys/time.h>
#include <vector>

// --CHANGES:BEGIN--
// includes are not needed
// #include "TouchSensor.h"
// --CHANGES:END--

#define TPA_NUM_PIXEL 8
#define TPA_DATA_SIZE 9

class ConfigParser;

typedef struct
{
    int theta;
    double probability;
    int value;
    double confidence;
    double diff;
    double lowpass;
    double lowpassconfidence;
    double mindiff;
    double maxdiff;
    double mean;
    double variance;
    double varconfidence;
    int span;
} tPyroSweepValue;

class SensorProcessing
{
    friend class SerialLine;

public:
    SensorProcessing(ConfigParser* config, bool processOdo = true);
    ~SensorProcessing();

    void setProcessOdo(bool p);

    void filterOdometry();
    void filterADXL();

    void startPyroFilter();
    void filterPyro(long time);

    void updateTime(timeval* time = NULL);

    long getRelativeTime_ms();

    double getInfraredDistance(int i);
    double getUltraSonicDistance(int i);
    double getTPA81Value(int sensor, int i);
    unsigned int getCompassHeading();
    unsigned int getCompassDistortion();
    unsigned int getCompass2Heading();
    unsigned int getPyroValue();
    int getPyroDirection();
    double getCO2Concentration();
    double getCO2Temperature();
    int getADXLValueX();
    int getADXLValueY();
    unsigned short getFlexValue(int i);
    unsigned short getBumperValue(int i);
    double getOdometryPosX();
    double getOdometryPosZ();
    double getOdometryPosTh();
    int getOdometryTransVel();
    int getOdometryRotVel();
    int getRID();

    void publishSensorData();
    void publishPyroSweepData(timeval starttime, long duration);

    int last_cube_heading;

protected:
    bool _processOdometry;
    ConfigParser* config;
    ros::Publisher pubOdom;
    tf::TransformBroadcaster odom_broadcaster;

    // --CHANGES:BEGIN--
    // object is not needed
    // TouchSensor touchSensor;
    // --CHANGES:END--

    unsigned short* infrared_distance;
    unsigned short* ultra_sonic_distance;

    unsigned short* tpa81_values;

    unsigned int compass_heading;
    unsigned int compass_distortion;
    unsigned int compass2_heading;
    unsigned int pyro_value;
    int pyro_direction;
    int adxl_x_value;
    int adxl_y_value;
    double adxl_averaged_x_value;
    double adxl_averaged_y_value;
    double adxl_x_variance;
    double adxl_y_variance;
    double adxl_filtered_x_value;
    double adxl_filtered_y_value;
    double* adxl_last_x_values;
    double* adxl_last_y_values;
    unsigned short* flex_value;
    unsigned short* bumper_value;

    // --CHANGES:BEGIN--
    // function is not needed
    // void fillBumperMessage(rescue_bumper_message & msg);
    // --CHANGES:END--

    void lurkerBumpers(int & left, int & middle, int & right);

    unsigned int co2_temperature;
    unsigned int co2_concentration;

    int odometry_left_front_dx;
    int odometry_left_front_vel;

    int odometry_left_rear_dx;
    int odometry_left_rear_vel;

    int odometry_right_front_dx;
    int odometry_right_front_vel;

    int odometry_right_rear_dx;
    int odometry_right_rear_vel;

    int rotation_vel;

    int cycle_time;

    double odometry_pos_x;
    double odometry_pos_y;

    //double odometry_pos_th;
    int odometry_vel_trans;
    int odometry_vel_rot;
    double odometry_last_dist;

    int pyro_last_angle;
    int pyro_last_value;
    long pyro_last_time;
    double pyro_lowpassed_value;
    double pyro_mindiff_value;
    double pyro_maxdiff_value;
    double pyro_var_min_value;
    double pyro_var_max_value;
    std::vector<tPyroSweepValue> pyrosweep_values;

    unsigned char stall;

    timeval currenttime;

    long starttime;
};

#endif
