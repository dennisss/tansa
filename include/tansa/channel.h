#ifndef TANSA_CHANNEL_H_
#define TANSA_CHANNEL_H_

#include <map>
#include <vector>
#include <iostream>

namespace tansa {

class Subscription {
public:
	Subscription(void (*func)(void *, void *), void *data) {
		this->func = func;
		this->data = data;
	}


	void call(void *raw) {
		func(raw, data);
	}

private:
	void (*func)(void *value, void *data);
	void *data;
};


template<class T, class V>
class ClassSubscription : public Subscription {

public:

	ClassSubscription(T* inst, void (T::*method)(const V*))
	 : Subscription(&ClassSubscription<T, V>::callClassMethod, this) {

		this->inst = inst;
		this->method = method;
	}

	T *inst;
	void (T::*method)(const V* value);


	static void callClassMethod(void *raw, void *data) {
		ClassSubscription *cs = (ClassSubscription *) data;

		V *val = (V *) raw;
		(cs->inst->*(cs->method))(val);
	}


};


class Channel {
public:

/*
	inline void subscribe(void (*func)(void *), void *data) {

		int ID = V::ID;
		if(listeners.count(ID) == 0) {
			listeners[ID] = std::vector<Subscription *>();
		}

		listeners[ID].push_back(new Subscription(func, data));
	}
*/

	template<class T, class V>
	inline void subscribe(void (T::*func)(const V*), T *inst) {

		int ID = V::ID;
		if(listeners.count(ID) == 0) {
			listeners[ID] = std::vector<Subscription *>();
		}

		listeners[ID].push_back(new ClassSubscription<T, V>(inst, func));
	}

protected:

	template<class V>
	inline void publish(const V &val) {

		int ID = V::ID;
		if(listeners.count(ID) == 0)
			return;

		for(auto subp : listeners[ID]) {
			subp->call((void *) &val);
		}

	}



private:


	std::map<int, std::vector<Subscription *>> listeners;
};


}


#endif
