#ifndef TANSA_CORE_H_
#define TANSA_CORE_H_

#include "json.hpp"
using json = nlohmann::json;

typedef void (*tansa_message_listener)(const json &data);

namespace tansa {


/**
 * Call this on program start before using the other functions of the library
 */
void init(bool enableMessaging);

void send_message(const json &msg);

void on_message(tansa_message_listener l);

}


#endif
