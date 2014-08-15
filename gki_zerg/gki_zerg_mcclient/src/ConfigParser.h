#ifndef __CONFIGPARSER_H__
#define __CONFIGPARSER_H__

#include <qstring.h>
#include <vector>

#include <ros/ros.h>

//#define WHEEL_OUTLINE 644 //mm New Wheels
//#define WHEEL_OUTLINE 393 //mm Smaller Wheels
#define WHEEL_TICKS_PER_CIRCULATION 7880
#define ROBOT_WIDTH 402 // Radabstand in mm

#define PYRO_APERTURE 20

#define ZERG 0
#define LURKER 1
/// Zergtilt will act as zerg to MC
#define ZERGTILT 2

class QDomElement;

typedef struct {
   float x;
   float y;
   float z;
   float pitch;
   float yaw;
   float roll;
} tGeomData;

enum ROBOT_TYPE {
   ZERG_TYPE,
   LURKER_TYPE
};

class ConfigParser {
   public:

      double tilt_min_angle;
      double tilt_max_angle;
      double tilt_default_angle;
      int tilt_min_pwm;
      int tilt_max_pwm;
      bool tilt_invert_servo;

      double pan_min_angle;
      double pan_max_angle;
      double pan_default_angle;
      int pan_min_pwm;
      int pan_max_pwm;
      bool pan_invert_servo;

      int arm_min_pos;
      int arm_max_pos;
      int arm_default_pos;
      int arm_min_pwm;
      int arm_max_pwm;
      bool arm_invert_servo;

      int lurker_arm_offset_front;
      int lurker_arm_offset_rear;

      ConfigParser();
      ~ConfigParser();

      bool parse( const QString &filename );
      QString configString();

      unsigned int numSensorsIR();
      unsigned int numSensorsUS();
      unsigned int numSensorsFlex();
      unsigned int numSensorsTPA81();
      unsigned int numSensorsBumper();

      double getTPA81SensorAngle(int i);

      bool enabledCompass() {
         return select_compass == 1;
      }

      bool enabledCompass2() {
         return select_compass2 == 1;
      }

      bool enabledPyro() {
         return select_pyro == 1;
      }

      bool enabledCO2() {
         return select_co2 == 1;
      }

      bool enabledAcc() {
         return select_acc == 1;
      }

      bool enabledOdometrylf() {
         return select_odometrylf == 1;
      }

      bool enabledOdometrylr() {
         return select_odometrylr == 1;
      }

      bool enabledOdometryrf() {
         return select_odometryrf == 1;
      }

      bool enabledOdometryrr() {
         return select_odometryrr == 1;
      }

      bool enabledOdometryrot() {
         return select_odometryrot == 1;
      }
      bool enabledStartButton() {
         return _startButtonActive;
      }
      unsigned char startButtonIr() {
         return _startButtonIr;
      }

      int getRobotType() {
         return robotType;
      }
      bool getIsZergTilt() {
         return isZergTilt;
      }

      double getWheelOutline() {
         return _wheelOutline;
      }

      double getMaxAcceleration() {
         return _maxAcceleration;
      }

      double getMinSpeed() {
         return _minSpeed;
      }
      double getMaxSpeed() {
         return _maxSpeed;
      }
      double getMinSpeedDist() {
         return _minSpeedDist;
      }
      double getMaxSpeedDist() {
         return _maxSpeedDist;
      }

      unsigned int numGeomData( int type );
      tGeomData getGeomData( int type, unsigned int i );
      void getBeamData( int type, float &minrange, float &maxrange, float &viewangle );
      void getTiltPositionerData(  double & min_angle, double & max_angle, int & min_pwm, int & max_pwm, double & default_angle , bool & invert_servo );
      void getPanPositionerData(  double & min_angle, double & max_angle, int & min_pwm, int & max_pwm, double & default_angle, bool& invert_servo);
      void getArmPositionerData(  int & min_pos, int & max_pos, int & min_pwm, int & max_pwm, int & default_pos, bool& invert_servo);

   protected:
      bool apply( QDomElement* tag );
      QString helpText();

      int robotType;
      bool isZergTilt;  ///< For small sensorhead: this is the tiltunit for the zerg (not the zerg itself)

      float ir_minrange;
      float ir_maxrange;
      float ir_viewangle;
      std::vector< unsigned char > ir_ad_converter;
      std::vector< unsigned char > ir_ad_ports;
      std::vector<tGeomData> ir_geom_data;

      bool _startButtonActive;
      unsigned char _startButtonIr; // start button is a IR to MC

      double _wheelOutline;
      double _maxAcceleration;
      double _minSpeed;
      double _maxSpeed;
      double _minSpeedDist;
      double _maxSpeedDist;

      float us_minrange;
      float us_maxrange;
      float us_viewangle;
      std::vector< unsigned char > us_iic_adresses;
      std::vector<tGeomData> us_geom_data;

      // TPA81 stuff
      float tpa81_minrange;
      float tpa81_maxrange;
      float tpa81_viewangle;
      std::vector< unsigned char > tpa_iic_adresses;
      std::vector<tGeomData> tpa81_geom_data;

      // CO2 stuff
      unsigned char select_co2;
      float co2_minrange;
      float co2_maxrange;
      float co2_viewangle;
      unsigned char co2_ad_converter1;
      unsigned char co2_ad_converter2;
      unsigned char co2_ad_port1;
      unsigned char co2_ad_port2;
      tGeomData co2_geom_data;

      // Compass stuff
      unsigned char select_compass;	
      unsigned char select_compass2;

      // Pyro stuff
      unsigned char select_pyro;
      float pyro_minrange;
      float pyro_maxrange;
      float pyro_viewangle;
      unsigned char pyro_ad_converter;
      unsigned char pyro_ad_port;
      tGeomData pyro_geom_data;

      unsigned char select_acc;

      // Flex
      std::vector< unsigned char > flex_ad_converter;
      std::vector< unsigned char > flex_ad_ports;

      std::vector<tGeomData> flex_geom_data;

      // Bumper
      std::vector< unsigned char > bumper_ad_converter;
      std::vector< unsigned char > bumper_ad_ports;
 
      unsigned char select_odometrylf;
      unsigned char select_odometrylr;
      unsigned char select_odometryrf;
      unsigned char select_odometryrr;
      unsigned char select_odometryrot;

      unsigned char left_motor_pid[3];
      unsigned char right_motor_pid[3];
      unsigned char rotation_params[5];

      unsigned char motor_stop_delay;
};

#endif

