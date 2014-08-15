#ifndef _SERIALLINE_H_
#define _SERIALLINE_H_

#include <qthread.h>

#include "qextserialport.h"

#include "ConfigParser.h"
#include "CommandQueue.h"

class SensorProcessing;

//bool g_mcInitialized;

class SerialLine : public QThread {
public:
	SerialLine( QextSerialPort* port, ConfigParser* config, CommandQueue* queue, SensorProcessing* sp );
	~SerialLine();

	virtual void run();

	void setTimeToExit() { timeToExit = true; }

	bool sendString( const QString &string );

	int initMCU();

	bool knock(); // Anklopfen

        int reinitialize(int level = 0);

	static int pan;
	static int tilt;
	static int arm;

protected:
	bool waitForCom(int msec); // Gibt TRUE zur�ck, falls ein Byte �ber serielle Schnittstelle empfangen wurde

	ConfigParser* config;
	CommandQueue* queue;
	QextSerialPort* serialPort;
	SensorProcessing* sensorProcessing;
	bool timeToExit;

        int _errorLevel;
        bool _knockFailed;

	QMutex mutex;
};


#endif
