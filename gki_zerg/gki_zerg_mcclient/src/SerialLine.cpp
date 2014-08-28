
#include "SensorProcessing.h"
#include "SerialLine.h"

#include <ros/ros.h>

//#define QUEUE_DISCHARGE 20
//#define MAX_VELOCITY_FREQ 2

bool g_mcInitialized = false;

int SerialLine::pan = 0;
int SerialLine::tilt = 0;
int SerialLine::arm = 0;

SerialLine::SerialLine( QextSerialPort* port, ConfigParser* config, CommandQueue* queue, SensorProcessing* sp ) {
	//config->parse( "config->xml" );
	serialPort = port;
	this->config = config;
	this->queue = queue;
	sensorProcessing = sp;
        pan = 0;
        tilt = 0;
        arm = 0;
        _errorLevel = 0;
        _knockFailed = false;

	timeToExit = false;
}

SerialLine::~SerialLine() {
}

int SerialLine::reinitialize(int level)
{
   int ret = 0;
   printf("Trying to initialize MC at level %d.\n", level);
   switch(level) {
      case 0:
         ret = initMCU();
         if(ret > 0)
            _errorLevel = 0;
         break;
      case 1:
         /*ret = initMCU();
         if(ret == 0) {
            printf("Level 1 fehlgeschlagen.\n");
            reinitialize(2);
         }
         if(ret > 0)
            _errorLevel = 0;
         break;*/
      case 2:
         serialPort->close();
         if( !serialPort->open(QIODevice::ReadWrite) ) {
            qWarning( "Level 2 fehlgeschlagen: Could not reopen serial port!" );
            return 0;
         }
         ret = initMCU();
         if(ret == 0) {
            printf("Level 2 fehlgeschlagen.\n");
         }
         if(ret == 2) {
            ret = initMCU();
            if(ret == 0) {
               printf("Level 2 fehlgeschlagen.\n");
            }
         }

         if(ret > 0)
            _errorLevel = 0;
         break;
      default:
         printf("Unknown Level: %d \n", level);
         return 0;
         break;
   }
   return ret;
}

/*	Initialisiert den MC.
RETURN:	0 ERROR
1 OK
2 RESTART
*/
int SerialLine::initMCU() {
   int ack = 0;
   char data = EOF;
   char msg[10];

   // Puffer freir�umen
   data = serialPort->getch();
   data = serialPort->getch();
   data = serialPort->getch();
   data = serialPort->getch();


   // send configuration to MC
   if(!sendString( config->configString() ))
      return 0;

   // wait for acknowledgment ("OK\0")


   while( ack < 10 && data != '\0' ) {
      data = serialPort->getch();

      if( data != EOF){
         msg[ack] = data;
         ack++;
      }
      /*if( ack == 0 && data == 'O' )
        ack++;

        if( ack == 1 && data == 'K' )
        ack++;

        if( ack == 2 && data == '\0' )
        ack++;*/
   }


   msg[9] = 0;

   printf( "MC Response: %s\n", msg );

   if(strcmp(msg, "OK") == 0) {
      _errorLevel = 0;
      return 1;
   }

   if(strcmp(msg, "RESET") == 0)
      return 2;

   return 0;
}

bool SerialLine::knock(){
   char answ[2];
   //sendString( QString( "R" ) );
   serialPort->putch( 'R' );

   msleep(1);

   if (!waitForCom(200)){
      printf("knocking send: Time out!\n");
      return false;
   }

   answ[0] = serialPort->getch(); // 'Y' einlesen

   if (!waitForCom(200)){
      printf("knocking get: Time out!\n");
      return false;
   }

   answ[1] = serialPort->getch(); // '\0' einlesen

   if(answ[0] != 'Y'){
      printf("knocking: not successful!\n");
      return false;
   }

   return true;
}

void SerialLine::run() {
   int num_bytes = 0;
   unsigned char checksum = 0xFF;
   int packets = 0;
   unsigned char buffer[1024];
   int res;

   res = reinitialize();
   if( res == 0 ) {
      qWarning( "Configuration of MCU failed!" );
      return;
   }

   if (res == 2){
      // Ein Restart muss durchgef�hrt werden:
      msleep(100);
      if (initMCU() == 0){
         qWarning( "Configuration of MCU failed!" );
         return;
      }
   }
   _errorLevel = 0;

   qWarning( "Configuration of MCU successful!" );
   g_mcInitialized = true;

   // --CHANGES:BEGIN--
   // function name changed
   // from getCurrentTime() to getCurrentTimeTS()
   //struct timeval _lastVelocityTime = getCurrentTimeTS();
   // --CHANGEs:END--

   while( !timeToExit ) {
errorHandling:
      if(_errorLevel > 0) {
         res = reinitialize(2);
         msleep(20);
      }

      //msleep(20);

      checksum = 0xFF;

      // process command queue
      //unsigned int q = 0;
      //unsigned int qsize= queue->size();
      while( queue->size() > 0 ) {
         QString command = queue->pop();

         // verhindere, dass zu haeufige Geschwindigkeitskommandos die Kommunikation behindern
         //if (command.at(3) == 'V' && TimeAgo(&_lastVelocityTime) < 1.0/MAX_VELOCITY_FREQ)
         //   continue;
         //if (command.at(3) == 'V')
         //   _lastVelocityTime = getCurrentTime();

         if( command.at( 3 ) == 'K' ) {
            command = command.left( 4 );
            unsigned char d;

            d = pan & 0x000000FF;
            command.append( QChar( d ) );
            d = (pan >> 8) & 0x000000FF;
            command.append( QChar( d ) );

            d = tilt & 0x000000FF;
            command.append( QChar( d ) );
            d = (tilt >> 8) & 0x000000FF;
            command.append( QChar( d ) );

            d = arm & 0x000000FF;
            command.append( QChar( d ) );
            d = (arm >> 8) & 0x000000FF;
            command.append( QChar( d ) );

            //printf("Tilt: %i, Pan:%i, Arm: %i\n", tilt, pan, arm);
         }

         //sendString( command.left( 1 ) );
         //msleep( 10 );
         if(!sendString( command.right( command.length() - 1 ) ) ) {
            break;
         }

         if( command.at( 3 ) == 'P' ) {
             ROS_ASSERT(false);
#if 0

            unsigned int k = 0;
            do {
               while( !waitForCom( 1 ) );

               buffer[k] = serialPort->getch();
               k++;

            } while ( buffer[k-1] != '\0' );

            ros::Time starttime, endtime;
            // TODO: REMOVE
            if( buffer[ k-3 ] == 'O' && buffer[ k-2 ] == 'K' ) {
               printf( "PyroSweep OK\n" );
               starttime = ros::Time::now();

               while( !waitForCom( 1 ) ); // wait until mcu ready

               for( k = 0; k < 724; k++ ) {
                  while( !waitForCom( 1 ) );

                  buffer[k] = serialPort->getch();
                  printf( "PyroSweep data received %i\n", k );
               }


               gettimeofday( &endtime, NULL );

               ros::Duration duration = endtime - starttime;

               k = 0;
               long currenttime;
               int i;
               for( i = 0; i < 181; i++ ) {
                  currenttime = duration * (k/2) / 362 ;

                  sensorProcessing->pyro_direction = i;
                  sensorProcessing->pyro_value = (unsigned char)buffer[k++];
                  sensorProcessing->pyro_value += ((unsigned short)buffer[k++]) << 8;
               }

               for( i = 180; i >= 0; i-- ) {
                  currenttime = duration * (k/2) / 362 ;

                  sensorProcessing->pyro_direction = i;
                  sensorProcessing->pyro_value = ((unsigned short)buffer[723-(k++ - 362)]) << 8;
                  sensorProcessing->pyro_value += (unsigned char)buffer[723-(k++ - 362)];
               }

               long st = starttime.tv_sec * 1000 + starttime.tv_usec / 1000;
               st += duration / 2;
               starttime.tv_sec = st / 1000;
               starttime.tv_usec = (st % 1000) * 1000;

            }
            else
               qWarning( "ERROR: no ok after pyro sweep command" );

            msleep( 50 );
#endif
         }  // if command.at(3) == 'P'
         else {

            unsigned int k = 0;
            do {
               if( !waitForCom( 30 ) ){
                  printf("\nKein OK empfangen\n");
                  _errorLevel = 1;

goto errorHandling;
               }

               buffer[k] = serialPort->getch();
               //printf("Antwort auf den Befehl: %i\n", buffer[k]);
               k++;

            } while ( buffer[k-1] != '\0' );

            // debug msg
            QString msg = "";
            for( unsigned int j = 0; j < k; j++ )
               msg.append( QChar( buffer[j] ) );

            qWarning( msg );

         }
      }  // while queue.size > 0


      msleep(10); // nicht zu oft Daten abfragen

      // trigger sensor send
      //sendString( QString( "R" ) );
      //msleep( 10 );
      sendString( QString( "" ) + QChar( 0 ) + QChar( 1 ) + QString( "S" ) );

      //qWarning( "sende Sensor Command" );

      if (!waitForCom(100)){
         printf("Time out!\n");
         continue;
      }

      buffer[0] = serialPort->getch();
      num_bytes = buffer[0] + 1;

      for(int k =1; k <= buffer[0]; k++){
         if( !waitForCom( 100 ) )
            break;

         buffer[k] = serialPort->getch();
      }

      for(int k=0; k < num_bytes - 2 ; k++){
         checksum = checksum ^ buffer[k];
      }

      if (!(checksum == buffer[num_bytes - 2] /*&& buffer[num_bytes - 1] == 19*/)){
         packets++;
         printf("Paket verloren %i\n", packets);
         for( int k = 0; k < num_bytes; k++ )
            printf( "%c ", buffer[ k ] );
         printf ("\n***********\n");
         _errorLevel = 1;
         msleep(100);
         continue;
      }

      /*		printf("Bytes: %i ", buffer[0] );
                        printf("%i ", buffer[1] );
                        printf("%i ", buffer[2] );
                        printf("%i ", buffer[3] );
                        printf("%i ", buffer[4] );
                        printf("%i ", buffer[5] );
                        printf("%\n************** \n ");
                        */

      sensorProcessing->updateTime();

      unsigned int i;
      int p = 1;
      for( i = 0; i < config->numSensorsIR(); i++ ) {
         sensorProcessing->infrared_distance[i] = (unsigned short)buffer[p++];
         sensorProcessing->infrared_distance[i] += ((unsigned short)buffer[p++]) << 8;
      }

      for( i = 0; i < config->numSensorsUS(); i++ ) {
         sensorProcessing->ultra_sonic_distance[i] = (unsigned short)buffer[p++];
         sensorProcessing->ultra_sonic_distance[i] += ((unsigned short)buffer[p++]) << 8;
      }

      for( i = 0; i < config->numSensorsTPA81() * TPA_DATA_SIZE; i++ ) {
         sensorProcessing->tpa81_values[i] = (unsigned short)buffer[p++];

         //printf("GET %i\n", sensorProcessing->tpa81_values[i]);
      }

      if( config->enabledCompass() ) {
         sensorProcessing->compass_heading = (unsigned short)buffer[p++];
         sensorProcessing->compass_heading += ((unsigned short)buffer[p++]) << 8;

         sensorProcessing->compass_distortion = (unsigned char)buffer[p++];
      }

      if( config->enabledCO2() ) {
         sensorProcessing->co2_concentration = (unsigned char)buffer[p++];
         sensorProcessing->co2_concentration += ((unsigned short)buffer[p++]) << 8;
         sensorProcessing->co2_temperature = (unsigned short)buffer[p++];
         sensorProcessing->co2_temperature += ((unsigned short)buffer[p++]) << 8;
      }

      if( config->enabledAcc() ) {
         sensorProcessing->adxl_x_value = (unsigned short)buffer[p++];
         sensorProcessing->adxl_x_value += ((unsigned short)buffer[p++]) << 8;

         sensorProcessing->adxl_y_value = (unsigned short)buffer[p++];
         sensorProcessing->adxl_y_value += ((unsigned short)buffer[p++]) << 8;
      }


      for( i = 0; i < config->numSensorsFlex(); i++ ) {
         sensorProcessing->flex_value[i] = (unsigned short)buffer[p++];
         sensorProcessing->flex_value[i] += ((unsigned short)buffer[p++]) << 8;
      }
      // HACK use flex as bumper
      for( i = 0; i < config->numSensorsBumper(); i++) {
         sensorProcessing->bumper_value[i] = (unsigned short)buffer[p++];
         sensorProcessing->bumper_value[i] += ((unsigned short)buffer[p++]) << 8;
      }

      if( config->enabledOdometrylf() ) {
         int raw;
         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_left_front_dx = (signed int) raw;

         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_left_front_vel = (signed int) raw;
      }

      if( config->enabledOdometrylr() ) {
         int raw;
         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_left_rear_dx = (signed int) raw;

         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_left_rear_vel = (signed int) raw;
      }

      if( config->enabledOdometryrf() ) {
         int raw;
         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_right_front_dx = (signed int) raw;

         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_right_front_vel = (signed int) raw;
      }

      if( config->enabledOdometryrr() ) {
         int raw;
         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_right_rear_dx = (signed int) raw;

         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->odometry_right_rear_vel = (signed int) raw;
      }

      if( config->enabledOdometryrot() ) {
         int raw;
         raw = (unsigned int)buffer[p++];
         raw += ((unsigned int)buffer[p++]) << 8;
         raw += ((unsigned int)buffer[p++]) << 16;
         raw += ((unsigned int)buffer[p++]) << 24;
         sensorProcessing->rotation_vel = (signed int) raw;

      }


      if( config->enabledCompass2() ) {
         sensorProcessing->compass2_heading = (unsigned short)buffer[p++];
         sensorProcessing->compass2_heading += ((unsigned short)buffer[p++]) << 8;
      }


      sensorProcessing->cycle_time = (unsigned short)buffer[p++];
      sensorProcessing->cycle_time += ((unsigned short)buffer[p++]) << 8;

      sensorProcessing->stall = (unsigned short)buffer[p++];

      //printf ("RotVel: %i, Left_front_Vel %i\n", sensorProcessing->rotation_vel, sensorProcessing->odometry_left_front_vel);

      /*
         printf ("Cycle Time: %i\n", sensorProcessing->cycle_time);
         printf ("DX: lf, lr, rf, rr: %i, %i, %i, %i\n", sensorProcessing->odometry_left_front_dx, sensorProcessing->odometry_left_rear_dx, sensorProcessing->odometry_right_front_dx, sensorProcessing->odometry_right_rear_dx);
         printf ("VEL: lf, lr, rf, rr: %i, %i, %i, %i\n", sensorProcessing->odometry_left_front_vel, sensorProcessing->odometry_left_rear_vel, sensorProcessing->odometry_right_front_vel, sensorProcessing->odometry_right_rear_vel);
         */

      sensorProcessing->filterOdometry();

      sensorProcessing->publishSensorData();

      msleep( 1 );
   }  // while(! timeToExit )

   serialPort->close();
}

bool SerialLine::waitForCom(int msec){
   int i =0;
   //while( serialPort->size() == 0 && i < msec){
   while( serialPort->bytesAvailable() <=0 && i < msec){
     msleep( 1 );
     i++;
   }
   //	printf("BW: %d\n",serialPort->bytesWaiting());

   if(i == msec)
      return false;
   else
      return true;
}

bool SerialLine::sendString( const QString &string ) {
   //	const char* str = string.ascii();
   //	unsigned int i;
   //	for( i = 0; i < string.length(); i++ ) {
   //		serialPort->putch( str[ i ] );
   //		msleep( 1 ); // for safety
   //    }

   if(knock()) {
      serialPort->writeBlock( string.ascii(), string.length() );
      return true;
   }
   _errorLevel = 1;
   return false;
}
