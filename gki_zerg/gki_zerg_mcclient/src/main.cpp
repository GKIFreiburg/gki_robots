#include <iostream>
#include <map>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

#include <assert.h>
#include <math.h>
#include <qstring.h>

#include <ros/ros.h>
#include "CommandQueue.h"
#include "SensorProcessing.h"
#include "paramHandling.h"

#include "gki_utils/timing.h"
#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <deque>
using std::deque;

#include "SerialLine.h"
extern bool g_mcInitialized;
#include "gki_utils/misc.h"


bool SHORT_DEBUG = false; // Debug mit einzelnen Chars

bool _newData = false;

bool ACTIVE_VELOCITY_LIMIT = false;

static ConfigParser* config;
static CommandQueue* queue;
static SensorProcessing* sensorProcessing;

static int _transVelLimit;
static int _rotVelLimit;
static int _lastTransVel;
static int _lastRotVel;
static ros::Time _lastVelTime;

static ros::Time _lastVelocityMessageTime;
// so lange wird eine Velocity Message hoechstens ausgefuert:
#define MAX_VELOCITY_MESSAGE_LIFETIME 0.5

// Prioritaeten der Befehle
#define CMD_PRIO_MOTOR 10
#define CMD_PRIO_VELOCITY 5
#define CMD_PRIO_RFIDACTUATOR 6
#define CMD_PRIO_CAM_CONTROL 5

#define NOT_DEFINED 999

// Bitmasken f�r den DigitalOut setzen
static const unsigned char leftMotorForward = 1 << 6;
static const unsigned char leftMotorBackward = 1 << 4;
static const unsigned char rightMotorForward = 1 << 2;
static const unsigned char rightMotorBackward = 1 << 0;
static const unsigned char frontArmPos = 1 << 1;
static const unsigned char frontArmNeg = 1 << 3;
static const unsigned char rearArmPos = 1 << 5;
static const unsigned char rearArmNeg = 1 << 7;
static double pan, tilt;
//static int camera_tilt;

static bool g_keepRunning = true;

#define CONF_PATH "/configs/"

static int initialHeading = NOT_DEFINED;
static bool s_MotorOn = true;

/*
static void quitproc(int signal)
{
   printf("Signal %d recvd.\n", signal);
   g_keepRunning = false;
}
*/

const unsigned int maxLastPitches = 20;

//static void setPanTilt(double pan_angle, double tilt_angle, int arm_pos);
//static int keepLaserPitch(double pitch);

static void msgInertiaCubeHandler(const sensor_msgs::Imu & msg)
{
    double yaw_deg = angles::to_degrees(tf::getYaw(msg.orientation));

    if (initialHeading == NOT_DEFINED)
        initialHeading = yaw_deg;
    double dyaw = yaw_deg - initialHeading;
    if(dyaw < -180)
        dyaw += 360;
    if(dyaw > 180)
        dyaw -= 360;
    int yaw = (int) dyaw;
    if(McClientParams::g_OdometryUseMagneticYaw)
        yaw = (int) yaw_deg; //RAD2DEG(msg.yaw);    TODO no difference in imu msg
    sensorProcessing->last_cube_heading = yaw;
}

static void setVelocity(int transVel, int rotVel)
{
  // Begrenzung gemaess Velocity Limit
  if (_transVelLimit >=0 && abs(transVel) > _transVelLimit)
      transVel = SIGN(transVel) * _transVelLimit;
   if (_rotVelLimit >= 0 && abs(rotVel) > _rotVelLimit)
      rotVel = SIGN(rotVel) * _rotVelLimit;

   ros::Time now = ros::Time::now();
   double max_trans_vel = fmax(0, _lastTransVel) +
                          (now - _lastVelTime).toSec()*config->getMaxAcceleration();
   if (transVel > max_trans_vel) {
       transVel = (int) max_trans_vel;
       printf("Begrenze Geschwindigkeit auf %.3f m/s\n", transVel/1000.0);
   }
   double min_trans_vel = fmin(0, _lastTransVel) -
                          (now - _lastVelTime).toSec()*config->getMaxAcceleration();
   if (transVel < min_trans_vel) {
       transVel = (int) min_trans_vel;
       printf("Begrenze Geschwindigkeit auf %.3f m/s\n", transVel/1000.0);
   }

   _lastTransVel = transVel;
   _lastRotVel = rotVel;

   _lastVelTime = ros::Time::now();

   int raw_trans_vel;
   int raw_rot_vel;
   unsigned char d;
   QString command = QString( "R" ) + QChar(0) + QChar( 5 ) + "V";

   // Umrechnungsfaktor von mm in wheel-ticks
   float ticksPerMm = (float)WHEEL_TICKS_PER_CIRCULATION /
     (float)config->getWheelOutline();

   raw_trans_vel = (int) ((float)transVel * ticksPerMm / 10.0);
   // Es wird durch 10 geteilt, damit der Wert in ein short passt

   // Verhindert Ueberlauf
   /*if (raw_trans_vel > 32767)
      raw_trans_vel = 32767;
   else if (raw_trans_vel < -32768)
      raw_trans_vel = -32768;*/
   // For some reason 6550 is overrun already
   if (raw_trans_vel > 6550)
      raw_trans_vel = 6550;
   else if (raw_trans_vel < -6550)
      raw_trans_vel = -6550;

   d = raw_trans_vel & 0x000000FF;
   command.append( QChar( d ) );
   d = (raw_trans_vel >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   // Umrechnungsfaktor von Grad/s in mm-differenz/s [right - left]
   float mmDiffPerGrad = M_PI/180.0 * (float)ROBOT_WIDTH;

   raw_rot_vel = (int) ((float)rotVel * mmDiffPerGrad * ticksPerMm / 10.0);
   // Es wird durch 10 geteilt, damit der Wert in ein short passt

   // Verhindert Ueberlauf
   if (raw_rot_vel > 32767)
      raw_rot_vel = 32767;
   else if (raw_rot_vel < -32768)
      raw_rot_vel = -32768;

   d = raw_rot_vel & 0x000000FF;
   command.append( QChar( d ) );
   d = (raw_rot_vel >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   //printf("Send velocity to MC: (tv, rv) = (%d, %d)\n",raw_trans_vel, raw_rot_vel);

   queue->push( CMD_PRIO_VELOCITY, command );
}


// HIER wird die Geschwindigkeit gesteuert
static void msgVelocityHandler(geometry_msgs::Twist msg)
{
   _lastVelocityMessageTime = ros::Time::now();

   if(!s_MotorOn /*|| McClientParams::g_Estop*/) {  // TODO estop
      msg.linear.x = 0.0;
      msg.angular.z = 0.0;
      ROS_INFO_THROTTLE(0.5, "Motor Off or estop - send (0, 0).\n");
      setVelocity((int)msg.linear.x,(int)msg.angular.z);
      return;
   }

   ROS_INFO_THROTTLE(1.0, "Received velocity command tv=%1.2lf, rv=%1.2lf",msg.linear.x, msg.angular.z);
   msg.linear.x *= 1000;
   msg.angular.z = angles::to_degrees(msg.angular.z)/0.7;   // factor by experiemnt
   setVelocity((int)msg.linear.x,(int)msg.angular.z);
}

// HIER wird die Geschwindigkeit gesteuert
#if 0
static void msgLowLevelVelocityHandler( MSG_INSTANCE msgRef, void* callData, void* clientData )
{
   (void) clientData;
   IPC_RETURN_TYPE err = IPC_OK;
   FORMATTER_PTR formatter;
   rescue_low_level_velocity_message lowLevelMsg;
   formatter = IPC_msgInstanceFormatter(msgRef);
   err = IPC_unmarshallData(formatter, callData, &lowLevelMsg, sizeof(lowLevelMsg));
   IPC_freeByteArray(callData);

   _lastVelocityMessageTime = getCurrentTimeTS();

   rescue_velocity_message msg;
   // HACK!!!
   msg.tv = (lowLevelMsg.velocityLeft + lowLevelMsg.velocityRight) / 2;
   msg.rv = (lowLevelMsg.velocityRight - lowLevelMsg.velocityLeft) / 2;

   if(!s_MotorOn || McClientParams::g_Estop) {
      msg.tv = 0.0;
      msg.rv = 0.0;
      DO_EVERY(0.5, printf("Motor Off or estop - send (0, 0).\n"); );
      setVelocity((int)msg.tv,(int)msg.rv);
      IPC_freeDataElements(formatter, &lowLevelMsg);
      return;
   }

   printf("Received velocity command tv=%1.2lf, rv=%1.2lf\n",msg.tv, msg.rv);
   msg.tv *= 5 * 1000;
   msg.rv = 10 * RAD2DEG(msg.rv);
   setVelocity((int)msg.tv,(int)msg.rv);
   IPC_freeDataElements(formatter, &lowLevelMsg);
}
#endif

#if 0
// Steuerung der Kameraaufhaengung
static void setPanTilt(double pan_angle, double tilt_angle, int arm_pos)
{
   QString command = QString( "R" ) + QChar(0) + QChar( 7 ) + QString( "K" );
   unsigned char d;

   int min_pwm = 1270;
   int max_pwm = 1895;


   int min_pwm_8B = 12;
   int max_pwm_8B = 27;

   // Tilt
   double MIN_ANGLE = -90.0;
   double MAX_ANGLE = 46.0;
   double defaultPos=0;

   // Pan
   double PAN_MIN_ANGLE = -90.0;
   double PAN_MAX_ANGLE = 90.0;
   double panDefaultPos=0;

   // Arm
   int ARM_MIN_POS = 0;
   int ARM_MAX_POS = 0;
   int armDefaultPos=0;

   bool tilt_invert_servo;
   bool pan_invert_servo;
   bool arm_invert_servo;

   config->getTiltPositionerData( MIN_ANGLE, MAX_ANGLE, min_pwm, max_pwm, defaultPos , tilt_invert_servo );
   config->getPanPositionerData( PAN_MIN_ANGLE, PAN_MAX_ANGLE, min_pwm_8B, max_pwm_8B, panDefaultPos, pan_invert_servo);
   //printf("MINA=%1.2lf MAXA=%1.2lf minPWM=%d maxPWM=%d \n",MIN_ANGLE, MAX_ANGLE, min_pwm, max_pwm);

   double pan  = pan_angle;
   if(pan < PAN_MIN_ANGLE)
      pan = PAN_MIN_ANGLE;
   if(pan > PAN_MAX_ANGLE)
      pan = PAN_MAX_ANGLE;

   double pan_angle_delta = pan - PAN_MIN_ANGLE;
   if(pan_invert_servo) {
      pan_angle_delta = PAN_MAX_ANGLE - pan;
   }

   int pan_pwm = min_pwm_8B + (int)( ((double)(max_pwm_8B - min_pwm_8B)) * (pan_angle_delta /(PAN_MAX_ANGLE - PAN_MIN_ANGLE)) );

   double tilt  = tilt_angle;
   if(tilt < MIN_ANGLE)
      tilt = MIN_ANGLE;
   if(tilt > MAX_ANGLE)
      tilt = MAX_ANGLE;

   if(tilt_invert_servo) {    // max und min vertauschen f�r bewegung in andere richtung
      double temp = MAX_ANGLE;
      MAX_ANGLE = MIN_ANGLE;
      MIN_ANGLE = temp;
   }

   //printf("Setting angle %1.2lf\n",tilt);
   int tilt_pwm = min_pwm + (int)( ((double)(max_pwm - min_pwm)) * (1.0 - (tilt - MIN_ANGLE)/(MAX_ANGLE - MIN_ANGLE)) );
   //printf("Setting PWM: %d\n",tilt_pwm);

   config->getArmPositionerData( ARM_MIN_POS, ARM_MAX_POS, min_pwm_8B, max_pwm_8B, armDefaultPos, arm_invert_servo);

   double arm  = arm_pos;
   if(arm < ARM_MIN_POS)
      arm = ARM_MIN_POS;
   if(arm > ARM_MAX_POS)
      arm = ARM_MAX_POS;

   if(arm_invert_servo) {    // max und min vertauschen f�r bewegung in andere richtung
      int temp = ARM_MAX_POS;
      ARM_MAX_POS = ARM_MIN_POS;
      ARM_MIN_POS = temp;
   }

   int arm_pwm = min_pwm_8B + (int)( ((double)(max_pwm_8B - min_pwm_8B)) * (1.0 - (arm - ARM_MIN_POS)/(ARM_MAX_POS - ARM_MIN_POS)) );

   d = pan_pwm & 0x000000FF;
   command.append( QChar( d ) );
   d = (pan_pwm >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   d = tilt_pwm & 0x000000FF;
   command.append( QChar( d ) );
   d = (tilt_pwm >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   d = arm_pwm & 0x000000FF;
   command.append( QChar( d ) );
   d = (arm_pwm >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   SerialLine::pan = pan_pwm;
   SerialLine::tilt = tilt_pwm;
   SerialLine::arm = arm_pwm;

   /*
      printf("camctrl, pan_angle: %f, tilt_angle: %f, arm angle: %f\n\tpan_pwm: %i tilt_pwm: %i arm_pwm %i\n",
      pan,
      tilt,
      arm,
      pan_pwm,
      tilt_pwm,
      arm_pwm);
      */

   queue->push( CMD_PRIO_CAM_CONTROL, command );

   rescue_tilt_ack_message msg;
   msg.timestamp = getCurrentTime();
   msg.host = getTechxHost();
   msg.angle = DEG2RAD(tilt);
   ComPublishToRobot(RESCUE_TILT_ACK_NAME, &msg);
}

void update(const pantilt_3dscan_message & msg, void* )
{
   if(msg.command == SCAN_3D_REQUEST) {
      // do 3dscan
      printf("Starting 3d scan...\n");
      double startAngle = McClientParams::g_3dScanStartAngle;
      double endAngle = McClientParams::g_3dScanEndAngle;
      double angleStep = McClientParams::g_3dScanAngleStep;

      setPanTilt(0, startAngle, 0);
      usleep(1000*1000);    // TODO: adjust this: wait until servo has reached startAngle

      pantilt_3dscan_message msgStart;
      prepareLocalMsgForSend(msgStart);
      msgStart.command = SCAN_3D_START;
      ComPublishToRobot(PANTILT_3DSCAN_MESSAGE_NAME, &msgStart);

      if(startAngle > endAngle) {
         for(double a = startAngle; a >= endAngle; a += angleStep) {
            setPanTilt(0, a, 0);
            usleep(100*1000);
         }
      } else {
         for(double a = startAngle; a <= endAngle; a += angleStep) {
            setPanTilt(0, a, 0);
            usleep(100*1000);
         }
      }

      pantilt_3dscan_message msgFinished;
      prepareLocalMsgForSend(msgFinished);
      msgFinished.command = SCAN_3D_FINISHED;
      ComPublishToRobot(PANTILT_3DSCAN_MESSAGE_NAME, &msgFinished);
   }
}
DEFINE_GLOBAL_HANDLER(msgHandler3dScan, pantilt_3dscan_message);

void update(const pantilt_set_tilt_message & msg, void* )
{
   double ang = RAD2DEG(msg.angle);
   setPanTilt(0, ang, 0);
}
DEFINE_GLOBAL_HANDLER(msgHandlerSetTilt, pantilt_set_tilt_message);
#endif

void printHelp() {
   printf("Usage:\n");
   printf("mcClient [-d <device>] -c <configname>\n");
   printf("-- Additional Options ---\n");
   printf("-a active velocity limit by laser\n");
   printf("-p control laser pitch angle\n");
   printf("-o publish odo2 instead of normal odo\n");
}

int main( int argc, char **argv )
{
   ros::init(argc, argv, "mcclient");
   ros::NodeHandle nh;
   _transVelLimit = -1; // -1: unlimited
   _rotVelLimit = -1;
   _lastTransVel = 0;
   _lastRotVel = 0;
   _lastVelocityMessageTime = ros::Time::now();
   std::string configName;
   std::string deviceName;
   bool controlLaserPitch = false;
   char c;


   while((c = getopt(argc, argv, "ad:c:hp")) != EOF)
   {
      switch(c)
      {
         case 'a':
            ACTIVE_VELOCITY_LIMIT = true;
            ROS_INFO("Using ACTIVE_VELOCITY_LIMIT.\n");
            break;
         case 'd':
            deviceName = optarg;
            break;
         case 'c':
            configName = optarg;
            break;
         case 'p':
            controlLaserPitch = true;
            break;
         case 'h':
         default:
            printHelp();
            exit(0);
            break;
      }
   }

   if(configName.empty()) {
      fprintf(stderr, "No configname given and could not autodetect configname.\n");
      printHelp();
      return 1;
   }
   QString configPath;
   //configPath += CONF_PATH;
   configPath += configName.c_str();
   printf("Config-File: %s\n", configPath.ascii());
   config = new ConfigParser();

   if(!config->parse( configPath.ascii() )) {
      fprintf(stderr, "Config Parsing error.\n");
      printHelp();
      return 1;
   }

   McClientParams::subscribeParams(true);

   ROS_INFO("Initializing serial device ...");

   queue = new CommandQueue();

   PortSettings settings;
   settings.FlowControl = FLOW_OFF;
   settings.BaudRate = BAUD38400;
   settings.DataBits = DATA_8;
   settings.Parity = PAR_NONE;
   settings.StopBits = STOP_1;
//   settings.Timeout_Sec = 0;
   settings.Timeout_Millisec = 500;

   if(deviceName.empty() && config->getRobotType() == ZERG) {
       ros::NodeHandle nhPriv("~");
       if(!nhPriv.getParam("device", deviceName)) {
           ROS_FATAL("No devicename given ...");
            printHelp();
            return 1;
       }
   }

   QextSerialPort* serialPort;

   if(deviceName.empty()) {
      fprintf(stderr, "No devicename given.\n");
      printHelp();
      return 1;
   }

   printf("\nUSING SERIAL INTERFACE %s\n", deviceName.c_str());
   serialPort = new QextSerialPort(deviceName.c_str(), settings );

   if(!serialPort->open(QIODevice::ReadWrite) ) {
      ROS_WARN( "ERROR: Could not open serial port!" );
      ROS_WARN( "correct command: mcClient -d ser_port -c configfile" );
      return 1;
   }

   // Signal Handling
   //signal(SIGINT, quitproc);
   //signal(SIGTERM, quitproc);     allow kill !!!

   sensorProcessing = new SensorProcessing( config , true);
   SerialLine serialLine( serialPort, config, queue, sensorProcessing );
   serialLine.start();

   std::cerr << "subs msg" << std::endl;
   ros::Subscriber subVel = nh.subscribe("cmd_vel", 1, msgVelocityHandler);
   //ComSubscribeToRobot( RESCUE_LOW_LEVEL_VELOCITY_NAME , &(msgLowLevelVelocityHandler), NULL );
   ros::Subscriber subImu = nh.subscribe("imu/data", 1, msgInertiaCubeHandler);
   //ComSubscribeToRobot( RESCUE_CAM_PWM_NAME, &(msgHandlerPwm), NULL);
   //ComSubscribeToRobot( PANTILT_3DSCAN_MESSAGE_NAME, msgHandler3dScan, NULL);
   //ComSubscribeToRobot( PANTILT_SET_TILT_NAME, msgHandlerSetTilt, NULL);

   //Initial Settings:
   pan = config->pan_default_angle;
   tilt = config->tilt_default_angle;

   while(!g_mcInitialized && ros::ok()) {
       ros::spinOnce();
         usleep(100*1000);
   }

   // Entering main loop
   while(ros::ok() && g_keepRunning)
   {
      //serialLine.run();
       ros::spinOnce();

      //printf("CycleTime: %.3fs\n", TimevalDiff(&tnow, &t));
      //t = tnow;

      // Stop, falls letzte Velocity Message zu alt

       ros::Time now = ros::Time::now();

      //// Adjust pitch angle ?
      //if (controlLaserPitch) {
      //   keepLaserPitch(McClientParams::g_KeepLaserPitch);
      //} else {
      //   setPanTilt(0.0, - McClientParams::g_KeepLaserPitch, 0.0);
      //}

      if ((now - _lastVelocityMessageTime).toSec() > MAX_VELOCITY_MESSAGE_LIFETIME &&
            (_lastTransVel != 0 || _lastRotVel != 0)) {
         setVelocity(0, 0);
         printf("STOP: Velocity Message Timeout");
      }
      usleep(50000);
   }

   printf("Closing mcClient. \n");

   serialLine.setTimeToExit();
   serialLine.terminate();

   delete serialPort;
   delete sensorProcessing;
   delete config;
   delete queue;
   printf("mcClient quit.\n");
   return 0;
}

#if 0
static int keepLaserPitch(double pitch)
{
   double lastPitch = 0;
   double lastPitchCount = 0;

   for(deque<double>::iterator it = s_LastPitches.begin(); it != s_LastPitches.end(); it++) {
      lastPitch += *it;
      lastPitchCount += 1.0;
   }
   if(lastPitchCount > 0)
      lastPitch /= lastPitchCount;

   lastPitch = RAD2DEG(lastPitch);

   DO_EVERY(1.0,
         cout << PVAR(lastPitch) << "\n";
         );

   double angle = -lastPitch-pitch;

   if (angle < -100) {
      angle = -100;
   }
   else if (angle > 45) {
      angle = 45;
   }

   DO_EVERY(1.0,
         cout << "moving laser to " << angle << "\n";
         );
   setPanTilt(0.0, angle, 0.0);

   return 0;
}
#endif


#if 0
static void msgHandlerPwm( MSG_INSTANCE msgInstance, void* callData, void* clientData )
{
   (void) msgInstance; // no warning
   (void) clientData; // no warning

   QString command = QString( "R" ) + QChar(0) + QChar( 7 ) + QString( "K" );
   rescue_cam_pwm_message* msg = (rescue_cam_pwm_message*)callData;
   unsigned char d;

   int tilt_pwm = msg->tilt_pwm;
   int pan_pwm = msg->pan_pwm;
   int arm_pwm = msg->arm_control_pwm;

   d = pan_pwm & 0x000000FF;
   command.append( QChar( d ) );
   d = (pan_pwm >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   d = tilt_pwm & 0x000000FF;
   command.append( QChar( d ) );
   d = (tilt_pwm >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   d = arm_pwm & 0x000000FF;
   command.append( QChar( d ) );
   d = (arm_pwm >> 8) & 0x000000FF;
   command.append( QChar( d ) );

   SerialLine::pan = pan_pwm;
   SerialLine::tilt = tilt_pwm;
   SerialLine::arm = arm_pwm;

   printf( "camctrl, pan_pwm: %d tilt_pwm: %d arm_pwm %d\n", pan_pwm, tilt_pwm, arm_pwm);

   queue->push( CMD_PRIO_CAM_CONTROL, command );
   //FORMATTER_PTR formatter = IPC_msgInstanceFormatter( msgInstance );
   //IPC_freeData( formatter, msg );
}
#endif
