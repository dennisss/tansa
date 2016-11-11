#include <tansa/core.h>
#include "sio_client.h"

#include <iostream>

using namespace std;

/*
	For talking to the gui and other applications
*/


static sio::client h;


void messaging_on_connected() {
	cout << "Connected!" << endl;
}



void messaging_init() {

	h.connect("http://127.0.0.1:3000");
	h.set_open_listener(messaging_on_connected);

}


namespace tansa {

void send_message(sio::message::list const& msglist) {
	sio::socket::ptr sock = h.socket();
	sock->emit("msg", msglist);
}

void on_message(tansa_message_listener l) {

	sio::socket::ptr sock = h.socket();

	sock->on("msg", sio::socket::event_listener_aux([&](string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp) {
		l(data);
	}));

}


}
