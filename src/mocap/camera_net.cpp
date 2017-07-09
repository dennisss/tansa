#include "camera_net.h"

#include <stdlib.h>
#include <string.h>


namespace tansa {


MocapCameraPacketParser::MocapCameraPacketParser() {
	pkt = (MocapCameraPacket *) malloc(0xffff + sizeof(MocapCameraPacket));
}

bool MocapCameraPacketParser::parse(const char *c) {

	if(state == 0) {
		if(*c == 'T')
			state = 1;
	}
	else if(state == 1) {
		if(*c == 'A')
			state = 2;
		else
			state = 0;
	}
	else if(state == 2) {
		pkt->type = *c;
		state = 3;
	}
	else if(state == 3 || state == 4) {
		uint8_t *s = (uint8_t  *) &pkt->size;
		s[state - 3] = *c;
		state++;

		if(pkt->size == 0 && state == 5) {
			state = 0;
			return true;
		}
	}
	else if(state >= 5) {
		pkt->data[state - 5] = *c;

		state++;
		if(state - 5 >= pkt->size) {
			state = 0;
			return true;
		}
	}


	return false;

}






}
