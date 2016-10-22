#ifndef TANSA_CORE_H_
#define TANSA_CORE_H_

#include <sio_message.h>

namespace tansa {


/**
 * Call this on program start before using the other functions of the library
 */
void init();

void send_message(sio::message::list const& msglist);

}


#endif
