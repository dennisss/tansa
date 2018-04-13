#ifndef TANSA_CORE_SUBSCRIPTION_H_
#define TANSA_CORE_SUBSCRIPTION_H_

#include "message.h"
#include "message_queue.h"


namespace tansa {

class Context;

class Subscription {
public:
	Subscription(Context *ctx, int msgId, void (*func)(void *, void *), void *data, unsigned size = 10) : queue(size) {
		this->ctx = ctx;
		this->msgId = msgId;
		this->func = func;
		this->data = data;
	}

	virtual ~Subscription() {}

	// TODO: The subscription should take ownership of the memory backing the message
	// TODO: It should also

	/**
	 * Distributes a message to all listeners
	 *
	 * @param raw a pointer to the dynamically allocated message. the subscription takes ownership of the memory backing it
	 */
	void post(Message::Ptr &msg) {
		queue.push(msg, false);
		//ctx->notify(); // For now moved to the Channel class
	}

	/**
	 * Distributes all available messages to callbacks
	 *
	 * Note: This should only be called internally in Context
	 */
	void call() {
		while(!queue.empty()) {
			Message::Ptr p;
			queue.pop(&p, false);
			Message *raw = p.get();

			func(raw, data);
		}
	}

	Context *ctx;
	int msgId;

private:
	MessageQueue<Message::Ptr> queue;
	void (*func)(void *value, void *data);
	void *data;
};


/**
 * A Subscription with the callback function being a class method
 */
template<class T, class V>
class ClassSubscription : public Subscription {

public:

	ClassSubscription(Context *ctx, int msgId, T* inst, void (T::*method)(const V*))
	 : Subscription(ctx, msgId, &ClassSubscription<T, V>::callClassMethod, this) {

		this->inst = inst;
		this->method = method;
	}

	virtual ~ClassSubscription() {}

private:

	T *inst;
	void (T::*method)(const V* value);

	static void callClassMethod(void *raw, void *data) {
		ClassSubscription *cs = (ClassSubscription *) data;

		V *val = (V *) raw;
		(cs->inst->*(cs->method))(val);
	}


};


}

#endif
