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
	cout << "..." << endl;
	sio::socket::ptr sock = h.socket();
	sock->emit("msg", msglist);
}

}
