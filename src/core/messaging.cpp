#include <tansa/core.h>
#include "sio_client.h"

#include <iostream>

using namespace std;

/*
	For talking to the gui and other applications
*/

namespace tansa {

static sio::client h;


void messaging_on_connected() {
	cout << "Connected!" << endl;
}



void messaging_init() {

	h.connect("http://127.0.0.1:4000");
	h.set_open_listener(messaging_on_connected);

}


void messaging_end() {
	h.close();
}

void send_message(const json &msg) {

	sio::message::ptr obj = sio::string_message::create(msg.dump());
	sio::message::list li(obj);

	sio::socket::ptr sock = h.socket();
	sock->emit("msg", li);
}

static tansa_message_listener listener;
void on_message(tansa_message_listener l) {

	listener = l;

	sio::socket::ptr sock = h.socket();

	// TODO: Make sure that this doesn't crash
	sock->on("msg", sio::socket::event_listener_aux([&](string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp) {
		// Convert string to json

		string str = data->get_string();
		json j = json::parse(str);

		listener(j);
	}));

}


}
