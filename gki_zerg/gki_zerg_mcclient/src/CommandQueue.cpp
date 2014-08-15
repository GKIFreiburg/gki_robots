#include "CommandQueue.h"

#include <algorithm>

CommandQueue::CommandQueue() {
	queue.clear();
}

CommandQueue::~CommandQueue() {
	queue.clear();
}

void CommandQueue::push( int priority, const QString &command ) {
	mutex.lock();
	QueueElement elem( priority, command );


	// Prüfen, ob ein Befehl dieses Typs schon vorhanden ist
	unsigned int i=0;
	while(i < queue.size()){
		if(queue[i].command.at(3) == command.at(3)){
			// Element entfernen
			queue.erase(queue.begin() + i);
		}

		i++;
	}

	queue.push_back( elem );
	mutex.unlock();
}

QString CommandQueue::pop() {
	mutex.lock();
	QString res = "";
	int unsigned i=0;
	int max = 0;
	int act_priority;
	int max_index=-1;


	while(i < queue.size()){
		act_priority = queue[i].priority;

		if (act_priority > max){
			max = act_priority;
			res = queue[i].command;
			max_index=i;
		}
		i++;
	}

	if(max_index>-1){
		queue.erase(queue.begin() + max_index);
	}

	mutex.unlock();
	return res;
}

void CommandQueue::clear() {
	mutex.lock();
	queue.clear();
	mutex.unlock();
}

unsigned int CommandQueue::size() {
	mutex.lock();
	unsigned int s = queue.size();
	mutex.unlock();
	return s;
}

