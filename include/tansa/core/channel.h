#ifndef TANSA_CORE_CHANNEL_H_
#define TANSA_CORE_CHANNEL_H_

#include "subscription.h"
#include "context.h"

#include <map>
#include <vector>

namespace tansa {



/**
 * An object which broadcasts messages
 */
class Channel {
public:

	/**
	 *
	 */
	template<class T, class V>
	inline void subscribe(Context *ctx, void (T::*func)(V*), T *inst) {

		int ID = V::ID;
		if(listeners.count(ID) == 0) {
			listeners[ID] = std::vector<Subscription *>();
		}

		Subscription *sub = new ClassSubscription<T, V>(ctx, inst, func);

		void *key = (void *) this;
		if(ctx->subs.count(key) == 0) {
			ctx->subs[key] = std::vector<Subscription *>();
		}

		ctx->subs[this].push_back(sub);

		listeners[ID].push_back(sub);
	}

	template<class V>
	inline void subscribe(Context *ctx, void (*func)(V*, void *), void *arg = NULL) {

		int ID = V::ID;
		if(listeners.count(ID) == 0) {
			listeners[ID] = std::vector<Subscription *>();
		}

		Subscription *sub = new Subscription(ctx, (void (*)(void *, void *)) func, arg);

		void *key = (void *) this;
		if(ctx->subs.count(key) == 0) {
			ctx->subs[key] = std::vector<Subscription *>();
		}

		ctx->subs[this].push_back(sub);

		listeners[ID].push_back(sub);
	}


protected:

	template<class V>
	inline void publish(V *val) {

		int ID = V::ID;
		if(listeners.count(ID) == 0)
			return;

		for(auto subp : listeners[ID]) {
			subp->post(val);
			subp->ctx->notify(); // TODO: Eventually move back to Subscription
		}

	}



private:


	std::map<int, std::vector<Subscription *>> listeners; /**< For each message id, this is all the entities that need to be noti */
};


}


#endif
