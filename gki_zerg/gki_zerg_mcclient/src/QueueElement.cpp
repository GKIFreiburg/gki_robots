#include "QueueElement.h"

QueueElement::QueueElement( unsigned int priority, const QString &command ) {
	this->priority = priority;
	this->command = command;
}

QueueElement::~QueueElement() {
}

