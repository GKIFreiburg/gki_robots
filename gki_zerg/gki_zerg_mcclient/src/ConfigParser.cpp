#include "ConfigParser.h"

#include <qdom.h>
#include <qfile.h>
#include <vector>
#include <stdio.h>

ConfigParser::ConfigParser() {
	// Default-Werte
	robotType = ZERG;
   isZergTilt = false;

	select_compass = 0;
	select_compass2 = 0;

	select_pyro = 0;
	pyro_ad_converter = 0;
	pyro_ad_port = 0;

	select_co2 = 0;
	co2_ad_converter1 = 0;
	co2_ad_converter2 = 0;
	co2_ad_port1 = 0;
	co2_ad_port2 = 0;

	select_acc = 0;

	select_odometrylf = 0;
	select_odometrylr = 0;
	select_odometryrf = 0;
	select_odometryrr = 0;
	select_odometryrot = 0;

	left_motor_pid[0] = 0;
	left_motor_pid[1] = 0;
	left_motor_pid[2] = 0;

	right_motor_pid[0] = 0;
	right_motor_pid[1] = 0;
	right_motor_pid[2] = 0;

	rotation_params[0] = 0;
	rotation_params[1] = 0;
	rotation_params[2] = 0;
	rotation_params[3] = 0;
	rotation_params[4] = 0;

        _wheelOutline = 500;
        _startButtonActive = false;
        _startButtonIr = 0;

    tilt_max_pwm = 2250;
    tilt_min_pwm = 800;
    tilt_max_angle = 65;
    tilt_default_angle=0;
    tilt_min_angle = - 85;
    tilt_invert_servo = false;
    pan_invert_servo = false;
    arm_invert_servo = false;

	motor_stop_delay = 0;
}

ConfigParser::~ConfigParser() {
}

bool ConfigParser::parse( const QString &filename ) {
	QDomDocument doc( "configfile" );
	QFile file( filename );
	if ( !file.open( QIODevice::ReadOnly ) ){
		printf("\nERROR: Could not open the configuration file!\n\n");
		return false;
	}
	if ( !doc.setContent( &file ) ) {
		qWarning( helpText() );
		file.close();
		return false;
	}
	file.close();

	QDomElement docElem = doc.documentElement();
	
	if( docElem.childNodes().count() == 0 )
		qWarning( helpText() );

	QDomNode n = docElem.firstChild();
	while( !n.isNull() ) {
		QDomElement e = n.toElement();
		if( !e.isNull() ) {
			if( !apply( &e ) ) {
				qWarning( "WARNING: Error parsing configuration file. Tag: "+e.tagName() );
			}
		}

		n = n.nextSibling();
	}

        return true;
}

bool ConfigParser::apply( QDomElement* tag ) {
	QString tagName = tag->tagName();

	// <? />
	// <help />
	if( tagName == "?" || tagName == "help" ) {
		qWarning( helpText() );
	}

	// <motor>
	//   <left p="1" i="0" d="0" />
	//   <right p="1" i="0" d="0" />
	// </motor>
	if( tagName == "motor" ) {
		QDomNode n = tag->firstChild();
		while( !n.isNull() ) {
			QDomElement e = n.toElement();
			if( !e.isNull() ) {
				if( e.tagName() == "left" ) {
					if( e.hasAttribute( "p" ) )
						left_motor_pid[0] = e.attribute( "p" ).toUShort();
					if( e.hasAttribute( "i" ) )
						left_motor_pid[1] = e.attribute( "i" ).toUShort();
					if( e.hasAttribute( "d" ) )
						left_motor_pid[2] = e.attribute( "d" ).toUShort();
				}
				if( e.tagName() == "right" ) {
					if( e.hasAttribute( "p" ) )
						right_motor_pid[0] = e.attribute( "p" ).toUShort();
					if( e.hasAttribute( "i" ) )
						right_motor_pid[1] = e.attribute( "i" ).toUShort();
					if( e.hasAttribute( "d" ) )
						right_motor_pid[2] = e.attribute( "d" ).toUShort();
				}

				if( e.tagName() == "delay" ) {
					if( e.hasAttribute( "cycles" ) )
						motor_stop_delay = e.attribute( "cycles" ).toUShort();
				}

				if( e.tagName() == "rotation" ) {
					if( e.hasAttribute( "p" ) )
						rotation_params[0] = e.attribute( "p" ).toUShort();
					if( e.hasAttribute( "i" ) )
						rotation_params[1] = e.attribute( "i" ).toUShort();
					if( e.hasAttribute( "d" ) )
						rotation_params[2] = e.attribute( "d" ).toUShort();
					if( e.hasAttribute( "offset" ) ){
						unsigned short val = e.attribute( "offset" ).toUShort();
						rotation_params[3] = (unsigned char) ( val & 0x00FF);
						rotation_params[4] = (unsigned char) (( val & 0xFF00) >> 8);
					}
				}
			}
			n = n.nextSibling();
		}
	}

	// <acc active="1" />
	else if( tagName == "acc" ) {
		if( tag->hasAttribute( "active" ) )
			select_acc = (unsigned char) tag->attribute( "active" ).toUShort();
		else
			select_acc = 1;
	}

	// <robot type="1"/>
	else if( tagName == "robot" ) {
		if( tag->hasAttribute( "type" ) ) {
			robotType = (unsigned char) tag->attribute( "type" ).toUShort();
         if(robotType == ZERGTILT) {
            robotType = ZERG;
            isZergTilt = true;
         }
      }
	}

        // <lurkerarmposoffset front="31" rear="77"/>
        else if( tagName == "lurkerarmposoffset" ) {
           if(tag->hasAttribute( "front" )) {
              lurker_arm_offset_front = -tag->attribute( "front" ).toInt();
           }
           if(tag->hasAttribute( "rear" )) {
              lurker_arm_offset_rear = -tag->attribute( "rear" ).toInt();
           }
        }

	// <bumper>
	//   <sensor converter="0" port="1" />
	// </bumper>
        else if( tagName == "bumper" ) {
           std::vector< unsigned char > converter;
           std::vector< unsigned char > ports;

           QDomNode n = tag->firstChild();
           while( !n.isNull() ) {
              QDomElement e = n.toElement();
              if( !e.isNull() ) {
                 if( e.tagName() == "sensor" ) {
                    if( e.hasAttribute( "converter" ) && e.hasAttribute( "port" ) ) {
                       converter.push_back( e.attribute( "converter" ).toUShort() );
                       ports.push_back( e.attribute( "port" ).toUShort() );
                    }
                 }
              }
              n = n.nextSibling();
           }

           bumper_ad_converter = converter;
           bumper_ad_ports = ports;
	}

	// <flex>
	//   <sensor converter="1" port="0" />
	// </flex>
	else if( tagName == "flex" ) {
		std::vector< unsigned char > converter;
		std::vector< unsigned char > ports;
		std::vector<tGeomData> geom_data;

		QDomNode n = tag->firstChild();
		while( !n.isNull() ) {
			QDomElement e = n.toElement();
			if( !e.isNull() ) {
				if( e.tagName() == "sensor" ) {
					if( e.hasAttribute( "converter" ) && e.hasAttribute( "port" ) ) {
						converter.push_back( e.attribute( "converter" ).toUShort() );
						ports.push_back( e.attribute( "port" ).toUShort() );
					}
	
					tGeomData gd = { 0, 0, 0, 0, 0, 0 };
					if( e.hasAttribute( "x" ) ) {
						gd.x = e.attribute( "x" ).toDouble();
					}
					if( e.hasAttribute( "y" ) ) {
						gd.y = e.attribute( "y" ).toDouble();
					}
					if( e.hasAttribute( "z" ) ) {
						gd.z = e.attribute( "z" ).toDouble();
					}
					if( e.hasAttribute( "pitch" ) ) {
						gd.pitch = e.attribute( "pitch" ).toDouble();
					}
					if( e.hasAttribute( "yaw" ) ) {
						gd.yaw = e.attribute( "yaw" ).toDouble();
					}
					if( e.hasAttribute( "roll" ) ) {
						gd.roll = e.attribute( "roll" ).toDouble();
					}
					geom_data.push_back( gd );

				}
			}
			n = n.nextSibling();
		}

		flex_ad_converter = converter;
		flex_ad_ports = ports;
                flex_geom_data = geom_data;
	}

	// <pyro active="1" converter="0" adport="1" pwmport="67" />
	else if( tagName == "pyro" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_pyro = tag->attribute( "active" ).toUShort();
		}
		else
			select_pyro = 1;
		
		if( tag->hasAttribute( "converter" ) )
			pyro_ad_converter = tag->attribute( "converter" ).toUShort();
		if( tag->hasAttribute( "adport" ) )
			pyro_ad_port = tag->attribute( "adport" ).toUShort();

		tGeomData gd = { 0, 0, 0, 0, 0, 0 };
		pyro_geom_data = gd;

		if( tag->hasAttribute( "minrange" ) )
			pyro_minrange = tag->attribute( "minrange" ).toFloat();
		if( tag->hasAttribute( "maxrange" ) )
			pyro_maxrange = tag->attribute( "maxrange" ).toFloat();
		if( tag->hasAttribute( "viewangle" ) )
			pyro_viewangle = tag->attribute( "viewangle" ).toFloat();

		if( tag->hasAttribute( "x" ) ) {
			pyro_geom_data.x = tag->attribute( "x" ).toDouble();
		}
		if( tag->hasAttribute( "y" ) ) {
			pyro_geom_data.y = tag->attribute( "y" ).toDouble();
		}
		if( tag->hasAttribute( "z" ) ) {
			pyro_geom_data.z = tag->attribute( "z" ).toDouble();
		}
		if( tag->hasAttribute( "pitch" ) ) {
			pyro_geom_data.pitch = tag->attribute( "pitch" ).toDouble();
		}
		if( tag->hasAttribute( "yaw" ) ) {
			pyro_geom_data.yaw = tag->attribute( "yaw" ).toDouble();
		}
		if( tag->hasAttribute( "roll" ) ) {
			pyro_geom_data.roll = tag->attribute( "roll" ).toDouble();
		}
	}

	// <co2 active="1" converter="0" adport="1" pwmport="67" />
	else if( tagName == "co2" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_co2 = tag->attribute( "active" ).toUShort();
		}
		else
			select_co2 = 1;
		
		if( tag->hasAttribute( "converter1" ) )
			co2_ad_converter1 = tag->attribute( "converter1" ).toUShort();
		if( tag->hasAttribute( "converter2" ) )
			co2_ad_converter2 = tag->attribute( "converter2" ).toUShort();
		if( tag->hasAttribute( "adport1" ) )
			co2_ad_port1 = tag->attribute( "adport1" ).toUShort();
		if( tag->hasAttribute( "adport2" ) )
			co2_ad_port2 = tag->attribute( "adport2" ).toUShort();

		tGeomData gd = { 0, 0, 0, 0, 0, 0 };
		co2_geom_data = gd;

		if( tag->hasAttribute( "minrange" ) )
			co2_minrange = tag->attribute( "minrange" ).toFloat();
		if( tag->hasAttribute( "maxrange" ) )
			co2_maxrange = tag->attribute( "maxrange" ).toFloat();
		if( tag->hasAttribute( "viewangle" ) )
			co2_viewangle = tag->attribute( "viewangle" ).toFloat();

		if( tag->hasAttribute( "x" ) ) {
			co2_geom_data.x = tag->attribute( "x" ).toDouble();
		}
		if( tag->hasAttribute( "y" ) ) {
			co2_geom_data.y = tag->attribute( "y" ).toDouble();
		}
		if( tag->hasAttribute( "z" ) ) {
			co2_geom_data.z = tag->attribute( "z" ).toDouble();
		}
		if( tag->hasAttribute( "pitch" ) ) {
			co2_geom_data.pitch = tag->attribute( "pitch" ).toDouble();
		}
		if( tag->hasAttribute( "yaw" ) ) {
			co2_geom_data.yaw = tag->attribute( "yaw" ).toDouble();
		}
		if( tag->hasAttribute( "roll" ) ) {
			co2_geom_data.roll = tag->attribute( "roll" ).toDouble();
		}
	}

	// <compass active="1" />
	else if( tagName == "compass" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_compass = tag->attribute( "active" ).toUShort();
		}
		else
			select_compass = 1;
	}

	// <compass2 active="1" />
	else if( tagName == "compass2" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_compass2 = tag->attribute( "active" ).toUShort();
		}
		else
			select_compass2 = 1;
	}

	// <odometrylf active="1" />
	else if( tagName == "odometrylf" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_odometrylf = tag->attribute( "active" ).toUShort();
		}
		else
			select_odometrylf = 0;
	}

	// <odometrylr active="1" />
	else if( tagName == "odometrylr" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_odometrylr = tag->attribute( "active" ).toUShort();
		}
		else
			select_odometrylr = 1;
	}

		// <odometryrf active="1" />
	else if( tagName == "odometryrf" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_odometryrf = tag->attribute( "active" ).toUShort();
		}
		else
			select_odometryrf = 0;
	}

	// <odometryrr active="1" />
	else if( tagName == "odometryrr" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_odometryrr = tag->attribute( "active" ).toUShort();
		}
		else
			select_odometryrr = 1;
	}

	// <odometryrot active="1" />
	else if( tagName == "odometryrot" ) {
		if( tag->hasAttribute( "active" ) ) {
			select_odometryrot = tag->attribute( "active" ).toUShort();
		}
		else
			select_odometryrot = 1;
	}

	// <sonar>
	//   <sensor adress="0xE0" />
	// </sonar>
	else if( tagName == "sonar" ) {
		std::vector< unsigned char > adresses;
		std::vector<tGeomData> geom_data;

		us_minrange = 0;
		us_maxrange = 0;
		us_viewangle = 0;
		if( tag->hasAttribute( "minrange" ) )
			us_minrange = tag->attribute( "minrange" ).toFloat();
		if( tag->hasAttribute( "maxrange" ) )
			us_maxrange = tag->attribute( "maxrange" ).toFloat();
		if( tag->hasAttribute( "viewangle" ) )
			us_viewangle = tag->attribute( "viewangle" ).toFloat();

		QDomNode n = tag->firstChild();
		while( !n.isNull() ) {
			QDomElement e = n.toElement();
			if( !e.isNull() ) {
				if( e.tagName() == "sensor" ) {
					if( e.hasAttribute( "adress" ) ) {
						bool ok = false;
						adresses.push_back( e.attribute( "adress" ).toUShort( &ok, 16 ) );
						if( !ok )
							qWarning( "Attribute adress not hexadecimal, tag sonar" );
					}
					else {
						qWarning( "No adress given for sonar sensor." );
						n = n.nextSibling();
						break;
					}
					
					tGeomData gd = { 0, 0, 0, 0, 0, 0 };
					if( e.hasAttribute( "x" ) ) {
						gd.x = e.attribute( "x" ).toDouble();
					}
					if( e.hasAttribute( "y" ) ) {
						gd.y = e.attribute( "y" ).toDouble();
					}
					if( e.hasAttribute( "z" ) ) {
						gd.z = e.attribute( "z" ).toDouble();
					}
					if( e.hasAttribute( "pitch" ) ) {
						gd.pitch = e.attribute( "pitch" ).toDouble();
					}
					if( e.hasAttribute( "yaw" ) ) {
						gd.yaw = e.attribute( "yaw" ).toDouble();
					}
					if( e.hasAttribute( "roll" ) ) {
						gd.roll = e.attribute( "roll" ).toDouble();
					}
					geom_data.push_back( gd );
				}
			}
			n = n.nextSibling();
		}

		us_iic_adresses = adresses;
		us_geom_data = geom_data;
	}

        // <tilt min_angle="-85" max_angle="65" min_pwm="800" max_pwm="2250" invertServo="1"/>
        else if( tagName == "tilt" ) {
           if( tag->hasAttribute( "min_angle" ) ) 
              tilt_min_angle = tag->attribute( "min_angle" ).toDouble();
           if( tag->hasAttribute( "max_angle" ) )
              tilt_max_angle = tag->attribute( "max_angle" ).toDouble();
           if( tag->hasAttribute( "min_pwm" ) )
              tilt_min_pwm = tag->attribute( "min_pwm" ).toShort();
           if( tag->hasAttribute( "max_pwm" ) )
              tilt_max_pwm = tag->attribute( "max_pwm" ).toShort();
           if( tag->hasAttribute( "default_angle" ) )
              tilt_default_angle = tag->attribute( "default_angle" ).toDouble();
           if( tag->hasAttribute( "invertServo" ) )
              tilt_invert_servo = (tag->attribute("invertServo").toInt() == 1);
        }

		else if( tagName == "pan" ) {
           if( tag->hasAttribute( "min_angle" ) ) 
              pan_min_angle = tag->attribute( "min_angle" ).toDouble();
           if( tag->hasAttribute( "max_angle" ) )
              pan_max_angle = tag->attribute( "max_angle" ).toDouble();
           if( tag->hasAttribute( "min_pwm" ) )
              pan_min_pwm = tag->attribute( "min_pwm" ).toShort();
           if( tag->hasAttribute( "max_pwm" ) )
              pan_max_pwm = tag->attribute( "max_pwm" ).toShort();
           if( tag->hasAttribute( "default_angle" ) )
              pan_default_angle = tag->attribute( "default_angle" ).toDouble();
           if( tag->hasAttribute( "invertServo" ) )
              pan_invert_servo = (tag->attribute("invertServo").toInt() == 1);
        }

		else if( tagName == "arm" ) {
           if( tag->hasAttribute( "min_pos" ) ) 
              arm_min_pos = (int) tag->attribute( "min_pos" ).toDouble();
           if( tag->hasAttribute( "max_pos" ) )
              arm_max_pos = (int) tag->attribute( "max_pos" ).toDouble();
           if( tag->hasAttribute( "min_pwm" ) )
              arm_min_pwm = tag->attribute( "min_pwm" ).toShort();
           if( tag->hasAttribute( "max_pwm" ) )
              arm_max_pwm = tag->attribute( "max_pwm" ).toShort();
           if( tag->hasAttribute( "default_pos" ) )
              arm_default_pos = (int) tag->attribute( "default_pos" ).toDouble();
           if( tag->hasAttribute( "invertServo" ) )
              arm_invert_servo = (tag->attribute("invertServo").toInt() == 1);
        }
        
        // <tpa81>
		else if( tagName == "tpa81" ) {
			std::vector< unsigned char > adresses;
			std::vector<tGeomData> geom_data;

			tpa81_minrange = 0;
			tpa81_maxrange = 0;
			tpa81_viewangle = 0;

			// Geom-Daten einlesen
			if( tag->hasAttribute( "minrange" ) )
				tpa81_minrange = tag->attribute( "minrange" ).toFloat();
			if( tag->hasAttribute( "maxrange" ) )
				tpa81_maxrange = tag->attribute( "maxrange" ).toFloat();
			if( tag->hasAttribute( "viewangle" ) )
				tpa81_viewangle = tag->attribute( "viewangle" ).toFloat();

			QDomNode n = tag->firstChild();
			while( !n.isNull() ) {
				QDomElement e = n.toElement();
				if( !e.isNull() ) {
					if( e.tagName() == "sensor" ) {
						if(e.hasAttribute("active")){
							if(e.attribute( "active" ).toUShort() == 1){
								if( e.hasAttribute( "adress" ) ) {
									bool ok = false;
									adresses.push_back( e.attribute( "adress" ).toUShort( &ok, 16 ) );
									if( !ok ){
										qWarning( "Attribute adress not hexadecimal, tag sonar" );
									}
								}
								else {
									qWarning( "No adress given for tpa81 sensor." );
									n = n.nextSibling();
									break;
								}
								tGeomData gd = { 0, 0, 0, 0, 0, 0 };
								if( e.hasAttribute( "x" ) ) {
									gd.x = e.attribute( "x" ).toDouble();
								}
								if( e.hasAttribute( "y" ) ) {
									gd.y = e.attribute( "y" ).toDouble();
								}
								if( e.hasAttribute( "z" ) ) {
									gd.z = e.attribute( "z" ).toDouble();
								}
								if( e.hasAttribute( "pitch" ) ) {
									gd.pitch = e.attribute( "pitch" ).toDouble();
								}
								if( e.hasAttribute( "yaw" ) ) {
									gd.yaw = e.attribute( "yaw" ).toDouble();
								}
								if( e.hasAttribute( "roll" ) ) {
									gd.roll = e.attribute( "roll" ).toDouble();
								}
								geom_data.push_back( gd );
							}

						}
					}
				}
				n = n.nextSibling();
			}

			tpa_iic_adresses = adresses;
			tpa81_geom_data = geom_data;
		}
        
        // <startbutton active="1" converter="0" port="0" />
        else if(tagName == "startbutton") {
           if(tag->hasAttribute("active") && tag->attribute("active").toInt() == 1) {
              if(tag->hasAttribute("useIr")) {
                 _startButtonActive = true;
                 _startButtonIr = tag->attribute("useIr").toUShort();
              } else {
                 ROS_WARN("Start button set active but no IR given.\n");
              }
           }
        }

        else if(tagName == "wheels") {
           if(tag->hasAttribute("outline")) {
              _wheelOutline = tag->attribute("outline").toDouble();
           } else {
              ROS_ERROR("No Wheel Outline defined in wheels.\n");
              exit(1);
           }
           if(tag->hasAttribute("acceleration")) {
              _maxAcceleration = tag->attribute("acceleration").toDouble();
           } else {
              ROS_ERROR("No Maximum Acceleration defined in wheels.\n");
              exit(1);
           }
        }

        else if(tagName == "speed") {
           if(tag->hasAttribute("min")) {
              _minSpeed = tag->attribute("min").toDouble();
           } else {
              ROS_ERROR("No Min Speed defined in speed.\n");
              exit(1);
           }
           if(tag->hasAttribute("max")) {
              _maxSpeed = tag->attribute("max").toDouble();
           } else {
              ROS_ERROR("No Max Speed defined in speed.\n");
              exit(1);
           }
           if(tag->hasAttribute("mindist")) {
              _minSpeedDist = tag->attribute("mindist").toDouble();
           } else {
              ROS_ERROR("No Min Speed Distance defined in speed.\n");
              exit(1);
           }
           if(tag->hasAttribute("maxdist")) {
              _maxSpeedDist = tag->attribute("maxdist").toDouble();
           } else {
              ROS_ERROR("No Max Speed Distance defined in speed.\n");
              exit(1);
           }
        }


	// <ir>
	//   <sensor converter="0" port="0" />
	// </ir>
	else if( tagName == "ir" ) {
		std::vector< unsigned char > converter;
		std::vector< unsigned char > ports;
		std::vector<tGeomData> geom_data;

		ir_minrange = 0;
		ir_maxrange = 0;
		ir_viewangle = 0;
		if( tag->hasAttribute( "minrange" ) )
			ir_minrange = tag->attribute( "minrange" ).toFloat();
		if( tag->hasAttribute( "maxrange" ) )
			ir_maxrange = tag->attribute( "maxrange" ).toFloat();
		if( tag->hasAttribute( "viewangle" ) )
			ir_viewangle = tag->attribute( "viewangle" ).toFloat();

		QDomNode n = tag->firstChild();
		while( !n.isNull() ) {
			QDomElement e = n.toElement();
			if( !e.isNull() ) {
				if( e.tagName() == "sensor" ) {
					if( e.hasAttribute( "converter" ) && e.hasAttribute( "port" ) ) {
						converter.push_back( e.attribute( "converter" ).toUShort() );
						ports.push_back( e.attribute( "port" ).toUShort() );
					}
					else {
						qWarning( "Port/converter not given for infrared sensor." );
						n = n.nextSibling();
						break;
					}
					
					tGeomData gd = { 0, 0, 0, 0, 0, 0 };
					if( e.hasAttribute( "x" ) ) {
						gd.x = e.attribute( "x" ).toDouble();
					}
					if( e.hasAttribute( "y" ) ) {
						gd.y = e.attribute( "y" ).toDouble();
					}
					if( e.hasAttribute( "z" ) ) {
						gd.z = e.attribute( "z" ).toDouble();
					}
					if( e.hasAttribute( "pitch" ) ) {
						gd.pitch = e.attribute( "pitch" ).toDouble();
					}
					if( e.hasAttribute( "yaw" ) ) {
						gd.yaw = e.attribute( "yaw" ).toDouble();
					}
					if( e.hasAttribute( "roll" ) ) {
						gd.roll = e.attribute( "roll" ).toDouble();
					}
					geom_data.push_back( gd );
				}
			}
			n = n.nextSibling();
		}

		ir_ad_converter = converter;
		ir_ad_ports = ports;
		ir_geom_data = geom_data;
	}

        else if( tagName == "scannerposition" ) {
            // do not need this in mcClient
        }

        else if( tagName == "cubeoffsets") {

        }
	else
		return false;

	return true;
}

QString ConfigParser::helpText() {
	QString res = "Possible tags:\nThe sensors are activated by default.\n\n";

	res += "Motor\n";
	res += "<motor>\n  <left p=\"1\" i=\"0\" d=\"0\" />\n  <right p=\"1\" i=\"0\" d=\"0\" <delay cycles=\"0\"/>\n</motor>\n\n";

	res += "Accelerometer\n";
	res += "<acc active=\"1\" />\n\n";

	res += "Flex Sensor\n";
	res += "<flex>\n  <sensor converter=\"1\" port=\"0\" />\n</flex>\n\n";

	res += "Pyro Sensor\n";
	res += "<pyro active=\"1\" converter=\"0\" adport=\"1\" />\n\n";

	res += "CO2 Sensor\n";
	res += "<co2 active=\"1\" converter=\"0\" adport1=\"1\" adport2=\"1\"/>\n\n";

	res += "Compass\n";
	res += "<compass active=\"1\" />\n\n";

	res += "Compass 2\n";
	res += "<compass2 active=\"1\" />\n\n";

	res += "Sonar Sensor\n";
	res += "<sonar>\n  <sensor adress=\"E0\" />\n</sonar>\n\n";

	res += "TPA81 Sensor\n";
	res += "<tpa81 minrange=\"0\" maxrange=\"0\" viewangle=\"0\">\n  <sensor adress=\"D0\" />\n</tpa81>\n\n";
       
         res += "Bumper Sensor \n";
         res += "<bumper>\n <sensor converter=\"0\" port=\"0\" />\n </bumper>\n\n";
        
	res += "IR Sensor\n";
	res += "<ir>\n  <sensor converter=\"0\" port=\"0\" />\n</ir>\n\n";

	res += "Odometry Sensors\n";
	res += "<odometry1 active=\"1\" />\n";
	res += "<odometry2 active=\"1\" />\n\n";

	return res;
}

// Bestimmt das Konfigurationspaket für den MC
QString ConfigParser::configString() {
	unsigned int i;

	QString sensorconfig = QString( "C" ) + QChar((char) robotType);

	sensorconfig += QChar( (char)ir_ad_converter.size() );

	for( i = 0; i < ir_ad_converter.size(); i++ ) {
		sensorconfig += QChar( (char)ir_ad_converter.at( i ) );
	}
	for( i = 0; i < ir_ad_ports.size(); i++ ) {
		sensorconfig += QChar( (char)ir_ad_ports.at( i ) );
	}

	sensorconfig += QChar( (char)us_iic_adresses.size() );
	for( i = 0; i < us_iic_adresses.size(); i++ ) {
		sensorconfig += QChar( (char)us_iic_adresses.at( i ) );
	}

	// TPA81 stuff
	sensorconfig += QChar( (char)tpa_iic_adresses.size() );
	for( i = 0; i < tpa_iic_adresses.size(); i++ ) {
		sensorconfig += QChar( (char)tpa_iic_adresses.at( i ) );
	}
        
	// CO2 sensor
	sensorconfig += QChar( (char)select_co2 );
	sensorconfig += QChar( (char)co2_ad_converter1 );
	sensorconfig += QChar( (char)co2_ad_converter2 );
	sensorconfig += QChar( (char)co2_ad_port1 );
	sensorconfig += QChar( (char)co2_ad_port2 );


	sensorconfig += QChar( (char)select_compass );

	// Pyro sensor
	sensorconfig += QChar( (char)select_pyro );
	sensorconfig += QChar( (char)pyro_ad_converter );
	sensorconfig += QChar( (char)pyro_ad_port );

	sensorconfig += QChar( (char)select_acc );

        // HACK add bumpers to flex to not reprogram MC firmware
	sensorconfig += QChar( (char)(flex_ad_converter.size() + bumper_ad_converter.size()));
	for( i = 0; i < flex_ad_converter.size(); i++ ) {
		sensorconfig += QChar( (char)flex_ad_converter.at( i ) );
	}
	for( i = 0; i < bumper_ad_converter.size(); i++ ) {
		sensorconfig += QChar( (char)bumper_ad_converter.at( i ) );
	}

	for( i = 0; i < flex_ad_ports.size(); i++ ) {
		sensorconfig += QChar( (char)flex_ad_ports.at( i ) );
	}
	for( i = 0; i < bumper_ad_ports.size(); i++ ) {
		sensorconfig += QChar( (char)bumper_ad_ports.at( i ) );
	}

	sensorconfig += QChar( (char)select_odometrylf );
	sensorconfig += QChar( (char)select_odometrylr );
	sensorconfig += QChar( (char)select_odometryrf );
	sensorconfig += QChar( (char)select_odometryrr );
	sensorconfig += QChar( (char)select_odometryrot );

	sensorconfig += QChar( (char)left_motor_pid[0] );
	sensorconfig += QChar( (char)left_motor_pid[2] );
	sensorconfig += QChar( (char)left_motor_pid[1] );

	sensorconfig += QChar( (char)right_motor_pid[0] );
	sensorconfig += QChar( (char)right_motor_pid[2] );
	sensorconfig += QChar( (char)right_motor_pid[1] );

	sensorconfig += QChar( (char)rotation_params[0] );
	sensorconfig += QChar( (char)rotation_params[2] );
	sensorconfig += QChar( (char)rotation_params[1] );
	sensorconfig += QChar( (char)rotation_params[3] );
	sensorconfig += QChar( (char)rotation_params[4] );

	sensorconfig += QChar( (char)select_compass2 );

	sensorconfig += QChar( (char)motor_stop_delay );

	return /*QString( "R" )+ */ (QString) QChar(0) + QChar( (char)(sensorconfig.length()) ) + sensorconfig;
}

unsigned int ConfigParser::numSensorsIR() {
	return ir_ad_converter.size();
}

unsigned int ConfigParser::numSensorsUS() {
	return us_iic_adresses.size();
}

unsigned int ConfigParser::numSensorsTPA81() {
	return tpa_iic_adresses.size();
}

unsigned int ConfigParser::numSensorsFlex() {
	return flex_ad_converter.size();
}

unsigned int ConfigParser::numSensorsBumper() {
   return bumper_ad_converter.size();
}

double ConfigParser::getTPA81SensorAngle(int i) {
      return i*5.12 - (8/2)*5.12;
}
   
unsigned int ConfigParser::numGeomData( int type ) {
	switch( type ) {
	  //case RESCUE_SONAR_TYPE:
	  //	return us_geom_data.size();
	  //case RESCUE_IR_TYPE:
	  //	return ir_geom_data.size();
	  //case RESCUE_PYRO_TYPE:
	  //	return 1;
	  //case RESCUE_CO2_TYPE:
	  //	return 1;
	  //case RESCUE_TPA81_TYPE:
	  //	return tpa81_geom_data.size();
	  //    case RESCUE_FLEX_TYPE:
	  //            return flex_geom_data.size();
		default:
			return 42;
	};
}

tGeomData ConfigParser::getGeomData( int type, unsigned int i ) {
	switch( type ) {
	  // case RESCUE_SONAR_TYPE:
	  //	return us_geom_data[i];
	  // case RESCUE_IR_TYPE:
	  //	return ir_geom_data[i];
	  // case RESCUE_PYRO_TYPE:
	  //	return pyro_geom_data;
	  // case RESCUE_TPA81_TYPE:
	  //	return tpa81_geom_data[i];
	  // case RESCUE_CO2_TYPE:
	  //	return co2_geom_data;
	  //    case RESCUE_FLEX_TYPE:
	  //            return flex_geom_data[i];
		default:
			break;
	};
	i=0;
	tGeomData dummy = { 0, 0, 0, 0, 0, 0 };
	return dummy;
}

void ConfigParser::getBeamData( int type, float &minrange, float &maxrange, float &viewangle ) {
	switch( type ) {
	  /*
		case RESCUE_SONAR_TYPE:
			minrange = us_minrange;
			maxrange = us_maxrange;
			viewangle = us_viewangle;
			break;
		case RESCUE_IR_TYPE:
			minrange = ir_minrange;
			maxrange = ir_maxrange;
			viewangle = ir_viewangle;
			break;
		case RESCUE_PYRO_TYPE:
			minrange = pyro_minrange;
			maxrange = pyro_maxrange;
			viewangle = pyro_viewangle;
			break;
		case RESCUE_CO2_TYPE:
			minrange = co2_minrange;
			maxrange = co2_maxrange;
			viewangle = co2_viewangle;
			break;
		case RESCUE_TPA81_TYPE:
			minrange = tpa81_minrange;
			maxrange = tpa81_maxrange;
			viewangle = tpa81_viewangle;
			break;
	  */
		default:
			minrange = 0;
			maxrange = 0;
			viewangle = 0;
			break;
	};
}


void ConfigParser::getTiltPositionerData(  double & min_angle, double & max_angle, int & min_pwm, int & max_pwm, double& default_angle, bool & invert_servo ) {
   min_angle = tilt_min_angle;
   max_angle = tilt_max_angle;
   min_pwm = tilt_min_pwm;
   max_pwm = tilt_max_pwm;
   default_angle = tilt_default_angle;
   invert_servo = tilt_invert_servo;
}

void ConfigParser::getPanPositionerData(  double & min_angle, double & max_angle, int & min_pwm, int & max_pwm, double& default_angle, bool& invert_servo) {
   min_angle = pan_min_angle;
   max_angle = pan_max_angle;
   min_pwm = pan_min_pwm;
   max_pwm = pan_max_pwm;
   default_angle = pan_default_angle;
   invert_servo = pan_invert_servo;
}

void ConfigParser::getArmPositionerData(  int & min_pos, int & max_pos, int & min_pwm, int & max_pwm, int & default_pos, bool& invert_servo){
   min_pos = arm_min_pos;
   max_pos = arm_max_pos;
   min_pwm = arm_min_pwm;
   max_pwm = arm_max_pwm;
   default_pos = arm_default_pos;
   invert_servo = arm_invert_servo;
}
