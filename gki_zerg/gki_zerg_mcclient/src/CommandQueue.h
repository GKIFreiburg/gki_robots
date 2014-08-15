#ifndef __COMMANDQUEUE_H__
#define __COMMANDQUEUE_H__

#include <vector>
#include <qstring.h>
#include <qmutex.h>

#include "QueueElement.h"

class CommandQueue {
public:
	CommandQueue();
	~CommandQueue();

	void push( int priority, const QString &command );
	QString pop();
	void clear();
	unsigned int size();

protected:
	std::vector< QueueElement > queue;

	QMutex mutex;
};


#endif

