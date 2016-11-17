#ifndef TANSA_CORE_H_
#define TANSA_CORE_H_

#include <sio_message.h>

typedef void (*tansa_message_listener)(sio::message::ptr const& data);

namespace tansa {


/**
 * Call this on program start before using the other functions of the library
 */
void init(bool enableMessaging);

void send_message(sio::message::list const& msglist);

}


#endif
