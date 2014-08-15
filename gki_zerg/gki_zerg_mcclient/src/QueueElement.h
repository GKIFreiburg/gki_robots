#ifndef __QUEUEELEMENT_H__
#define __QUEUEELEMENT_H__

#include <qstring.h>
#include <functional>

class QueueElement {
public:

	QueueElement( unsigned int priority, const QString &command );
	~QueueElement();

	unsigned int priority;
	QString command;
};

struct queue_less_than : public std::binary_function< QueueElement, QueueElement, bool> {
	bool operator()( QueueElement x, QueueElement y ) { 
		return x.priority < y.priority;
	}
};

#endif

