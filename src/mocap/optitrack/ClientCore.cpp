#include "ClientCore.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <poll.h>

#include <arpa/inet.h>



#define MULTICAST_ADDRESS		"239.255.42.99"     // IANA, local network
#define PORT_COMMAND            1510
#define PORT_DATA  			    1511

#define DEBUG(...)
//printf(__VA_ARGS__)




// TODO: Make sure that the version of the server is determined before reading data
int NatNetVersion[4] = {0,0,0,0};
int ServerVersion[4] = {0,0,0,0};


#include <iostream>
using namespace std;




ClientCore::ClientCore(){

	running = false;

	frame = NULL;
	frameMem = NULL;
	dataDescs = NULL;


	data_callback = NULL;
	data_callback_arg = NULL;
}


ClientCore::~ClientCore(){

	if(running){
		stop();
	}
}



int create_socket(){
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	#ifdef SO_REUSEPORT
	int reuse = 1;
	if(setsockopt(sock, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(int)) == -1){
		printf("Failed to make socket reusable\n");
		exit(1);
		return -1;
	}
	#endif

	return sock;
}




void ClientCore::start(const char *clientAddr, const char *serverAddr){


	int optval;

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));


	this->cmd_socket = create_socket();

	// Enable broadcasting
	optval = 1;
	if(setsockopt(this->cmd_socket, SOL_SOCKET, SO_BROADCAST, (void *)&optval, sizeof(optval)) == -1) {
		perror("setsockopt (SO_BROADCAST)");
		exit(1);
	}

	// Bind to command address
	sa.sin_port = htons(PORT_COMMAND);
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY); // TODO: This should be the local interface ip
	if(::bind(this->cmd_socket, (const struct sockaddr*)&sa, sizeof(sa)) < 0){
		printf("ClientCore: failed to bind command socket\n");
	}





	this->data_socket = create_socket();

	// Bind to data address
	sa.sin_port = htons(PORT_DATA);
	sa.sin_family = AF_INET;
	sa.sin_addr.s_addr = htonl(INADDR_ANY); // TODO: This should be the interface ip
	if(::bind(this->data_socket, (const struct sockaddr*)&sa, sizeof(sa)) < 0){
		printf("ClientCore: failed to bind data socket\n");
	}


	// Join multicast group
	ip_mreq mreq;

	inet_pton(AF_INET, MULTICAST_ADDRESS,  &mreq.imr_multiaddr); //&this->multi_addr.sin_addr);
	//mreq.imr_multiaddr = this->multi_addr.sin_addr;

	int r;

	inet_pton(AF_INET, clientAddr, &mreq.imr_interface.s_addr);
	//mreq.imr_interface.s_addr = htonl(INADDR_ANY);
	r = setsockopt(this->data_socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, (void *)&mreq, sizeof(mreq)); // TODO: Error check this

	if(r < 0){
		printf("Failed to join multicast group!\n");
	}


	// Make the buffer bigger
	optval = 0x100000;
	setsockopt(this->data_socket, SOL_SOCKET, SO_RCVBUF, (void *)&optval, sizeof(optval)); // TODO: Error check this




	running = true;

	cout << "Starting threads..." << endl;

	// Start data server
	if(pthread_create(&this->data_thread, NULL, data_server, this) != 0){
		printf("ClientCore: failed to create data thread\n");
	}

	// Start command server
	if(pthread_create(&this->cmd_thread, NULL, cmd_server, this) != 0){
		printf("ClientCore: failed to create command thread\n");
	}


}

void ClientCore::stop(){
	this->running = false;

	// Close the sockets
	close(this->data_socket);
	close(this->cmd_socket);


	// Join the threads
	pthread_join(this->data_thread, NULL);
	pthread_join(this->cmd_thread, NULL);
}




void *data_server(void *arg){
	ClientCore *cc = (ClientCore *)arg;

	char szData[20000];
	struct sockaddr_in sa;
	socklen_t addr_len = sizeof(struct sockaddr_in);

	while(cc->running){
		int r = recvfrom(cc->data_socket, szData, sizeof(szData), 0, (sockaddr *)&sa, &addr_len);

		if(r == 0){
			continue;
		}
		else if(r < 0){
			continue;
		}

//		cout << "Got data" << endl;

		cc->unpack(szData);
	}
}

void *cmd_server(void *arg){
	ClientCore *cc = (ClientCore *)arg;


	int r;
	struct sockaddr_in sa;
	socklen_t addr_len = sizeof(struct sockaddr);
	sPacket PacketIn;

    while(cc->running){
        // blocking
        r = recvfrom(cc->cmd_socket, (char *)&PacketIn, sizeof(sPacket), 0, (struct sockaddr *)&sa, &addr_len);

        if((r == 0) || (r < 0))
            continue;

		// debug - print message
		//sprintf(str, "[Client] Received command from %d.%d.%d.%d: Command=%d, nDataBytes=%d",
		//		TheirAddress.sin_addr.S_un.S_un_b.s_b1, TheirAddress.sin_addr.S_un.S_un_b.s_b2,
		//		TheirAddress.sin_addr.S_un.S_un_b.s_b3, TheirAddress.sin_addr.S_un.S_un_b.s_b4,
		//		(int)PacketIn.iMessage, (int)PacketIn.nDataBytes);


        // handle command
		switch(PacketIn.iMessage){
		case NAT_MODELDEF:
			cc->unpack((char*)&PacketIn);
			break;
		case NAT_FRAMEOFDATA:
			cc->unpack((char*)&PacketIn);
			break;
		case NAT_PINGRESPONSE:
			for(int i = 0; i < 4; i++){
				NatNetVersion[i] = (int)PacketIn.Data.Sender.NatNetVersion[i];
				ServerVersion[i] = (int)PacketIn.Data.Sender.Version[i];
			}
			break;
		case NAT_RESPONSE:
			//gCommandResponseSize = PacketIn.nDataBytes;
			//if(gCommandResponseSize==4)
			//	memcpy(&gCommandResponse, &PacketIn.Data.lData[0], gCommandResponseSize);
			//else{
			//	memcpy(&gCommandResponseString[0], &PacketIn.Data.cData[0], gCommandResponseSize);
			//	printf("Response : %s", gCommandResponseString);
			//	gCommandResponse = 0;   // ok
			//}
			break;
		case NAT_UNRECOGNIZED_REQUEST:
			printf("[Client] received 'unrecognized request'\n");
			//gCommandResponseSize = 0;
			//gCommandResponse = 1;       // err
			break;
		case NAT_MESSAGESTRING:
			printf("[Client] Received message: %s\n", PacketIn.Data.szData);
			break;
		}
	}




	return NULL;
}




void ClientCore::sendPacket(sPacket *packet){

	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));

	sa.sin_family = AF_INET;
	sa.sin_port = htons(PORT_COMMAND);
	sa.sin_addr.s_addr = INADDR_BROADCAST; // TODO: This should be the address of the server

	int res = sendto(this->cmd_socket, &packet, 4 + packet->nDataBytes, 0, (struct sockaddr *)&sa, sizeof(struct sockaddr_in));


	if(res == -1){
		// Error
	}

}


void ClientCore::ping(){
	sPacket PacketOut;
	PacketOut.iMessage = NAT_PING;
	PacketOut.nDataBytes = 0;

	this->sendPacket(&PacketOut);
}









void ClientCore::unpack(char* pData){

	int major = NatNetVersion[0];
	int minor = NatNetVersion[1];

	char *ptr = pData;

	DEBUG("Begin Packet\n-------\n");

	// message ID
	int MessageID = 0;
	memcpy(&MessageID, ptr, 2); ptr += 2;
	DEBUG("Message ID : %d\n", MessageID);

	// size
	int nBytes = 0;
	memcpy(&nBytes, ptr, 2); ptr += 2;
	DEBUG("Byte count : %d\n", nBytes);

	if(MessageID == NAT_FRAMEOFDATA){ // FRAME OF MOCAP DATA packet
		this->unpackFrame(ptr);
	}
	else if(MessageID == NAT_MODELDEF){ // Data Descriptions
		this->unpackDataDescs(ptr);
	}
	else{
		DEBUG("Unrecognized Packet Type.\n");
	}

}

void ClientCore::unpackFrame(char *ptr){

	int major = 2, minor = 9;


	Memory *mem = new Memory();
	sFrameOfMocapData *frame = (sFrameOfMocapData *)mem->alloc(sizeof(sFrameOfMocapData));
	memset(frame, 0, sizeof(sFrameOfMocapData));



	// frame number
	int frameNumber = 0; memcpy(&frame->iFrame, ptr, 4); ptr += 4;
	DEBUG("Frame # : %d\n", frame->iFrame);


	// number of data sets (markersets, rigidbodies, etc)
	int nMarkerSets = 0; memcpy(&nMarkerSets, ptr, 4); ptr += 4;
	DEBUG("Marker Set Count : %d\n", nMarkerSets);
	frame->nMarkerSets = nMarkerSets;

	for(int i = 0; i < nMarkerSets; i++){

		sMarkerSetData &set = frame->MocapData[i];

		// Markerset name
		strcpy(set.szName, ptr);
		int nDataBytes = (int)strlen(set.szName) + 1;
		ptr += nDataBytes;
		DEBUG("Model Name: %s\n", set.szName);

		// marker data
		int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
		DEBUG("Marker Count : %d\n", nMarkers);
		set.nMarkers = nMarkers;
		set.Markers = (MarkerData *)mem->alloc(sizeof(MarkerData) * nMarkers);

		// TODO: Just do a bulk memcpy
		for(int j = 0; j < nMarkers; j++){
			MarkerData &m = set.Markers[j];
			memcpy(&m, ptr, 12); ptr += 12;
			DEBUG("\tMarker %d : [x=%3.2f,y=%3.2f,z=%3.2f]\n", j, m[0], m[1], m[2]);
		}
	}

	// unidentified markers
	int nOtherMarkers = 0; memcpy(&nOtherMarkers, ptr, 4); ptr += 4;
	DEBUG("Unidentified Marker Count : %d\n", nOtherMarkers);
	frame->nOtherMarkers = nOtherMarkers;
	frame->OtherMarkers = (MarkerData *)mem->alloc(sizeof(MarkerData) * nOtherMarkers);

	// TODO: Just do a bulk memcpy
	for(int j = 0; j < nOtherMarkers; j++){
		MarkerData &m = frame->OtherMarkers[j];
		memcpy(&m, ptr, 12); ptr += 12;
		DEBUG("\tMarker %d : pos = [%3.2f,%3.2f,%3.2f]\n", j, m[0], m[1], m[2]);
	}

	// rigid bodies
	int nRigidBodies = 0;
	memcpy(&nRigidBodies, ptr, 4); ptr += 4;
	DEBUG("Rigid Body Count : %d\n", nRigidBodies);
	frame->nRigidBodies = nRigidBodies;

	for(int j = 0; j < nRigidBodies; j++){

		sRigidBodyData &body = frame->RigidBodies[j];

		// rigid body pos/ori
		memcpy(&body.ID, ptr, 4); ptr += 4;
		memcpy(&body.x, ptr, 4); ptr += 4;
		memcpy(&body.y, ptr, 4); ptr += 4;
		memcpy(&body.z, ptr, 4); ptr += 4;
		memcpy(&body.qx, ptr, 4); ptr += 4;
		memcpy(&body.qy, ptr, 4); ptr += 4;
		memcpy(&body.qz, ptr, 4); ptr += 4;
		memcpy(&body.qw, ptr, 4); ptr += 4;
		DEBUG("ID : %d\n", body.ID);
		DEBUG("pos: [%3.2f,%3.2f,%3.2f]\n", body.x, body.y, body.z);
		DEBUG("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", body.qx, body.qy, body.qz, body.qw);


		// associated marker positions
		int nRigidMarkers = 0; memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
		DEBUG("Marker Count: %d\n", nRigidMarkers);
		int nBytes = nRigidMarkers * sizeof(MarkerData);
		MarkerData* markerData = (MarkerData *)mem->alloc(nBytes);
		memcpy(markerData, ptr, nBytes);
		ptr += nBytes;
		body.nMarkers = nRigidMarkers;
		body.Markers = markerData;

		if(major >= 2){
			// associated marker IDs
			nBytes = nRigidMarkers*sizeof(int32_t);
			int32_t* markerIDs = (int32_t*)mem->alloc(nBytes);
			memcpy(markerIDs, ptr, nBytes);
			ptr += nBytes;
			body.MarkerIDs = markerIDs;



			// associated marker sizes
			nBytes = nRigidMarkers*sizeof(float);
			float* markerSizes = (float*)mem->alloc(nBytes);
			memcpy(markerSizes, ptr, nBytes);
			ptr += nBytes;
			body.MarkerSizes = markerSizes;

			for(int k = 0; k < nRigidMarkers; k++){
				DEBUG("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
			}

			//if(markerIDs)
			//	free(markerIDs);
			//if(markerSizes)
			//	free(markerSizes);

		}
		else{
			for(int k = 0; k < nRigidMarkers; k++){
				DEBUG("\tMarker %d: pos = [%3.2f,%3.2f,%3.2f]\n", k, markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
			}
		}
		//if(markerData)
		//	free(markerData);

		if(major >= 2){
			// Mean marker error
			memcpy(&body.MeanError, ptr, 4); ptr += 4;
			DEBUG("Mean marker error: %3.2f\n", body.MeanError);
		}

		// 2.6 and later
		if(((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0)){
			// params
			memcpy(&body.params, ptr, 2); ptr += 2;
			//bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
		}

	} // next rigid body


	// skeletons (version 2.1 and later)
	if(((major == 2)&&(minor>0)) || (major>2)){
		int nSkeletons = 0;
		memcpy(&nSkeletons, ptr, 4); ptr += 4;
		DEBUG("Skeleton Count : %d\n", nSkeletons);
		for (int j=0; j < nSkeletons; j++){
			// skeleton id
			int skeletonID = 0;
			memcpy(&skeletonID, ptr, 4); ptr += 4;
			// # of rigid bodies (bones) in skeleton
			int nRigidBodies = 0;
			memcpy(&nRigidBodies, ptr, 4); ptr += 4;
			DEBUG("Rigid Body Count : %d\n", nRigidBodies);
			for (int j = 0; j < nRigidBodies; j++){
				// rigid body pos/ori
				int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
				float x = 0.0f; memcpy(&x, ptr, 4); ptr += 4;
				float y = 0.0f; memcpy(&y, ptr, 4); ptr += 4;
				float z = 0.0f; memcpy(&z, ptr, 4); ptr += 4;
				float qx = 0; memcpy(&qx, ptr, 4); ptr += 4;
				float qy = 0; memcpy(&qy, ptr, 4); ptr += 4;
				float qz = 0; memcpy(&qz, ptr, 4); ptr += 4;
				float qw = 0; memcpy(&qw, ptr, 4); ptr += 4;
				DEBUG("ID : %d\n", ID);
				DEBUG("pos: [%3.2f,%3.2f,%3.2f]\n", x, y, z);
				DEBUG("ori: [%3.2f,%3.2f,%3.2f,%3.2f]\n", qx,qy,qz,qw);

				// associated marker positions
				int nRigidMarkers = 0;  memcpy(&nRigidMarkers, ptr, 4); ptr += 4;
				printf("Marker Count: %d\n", nRigidMarkers);
				int nBytes = nRigidMarkers*3*sizeof(float);
				float* markerData = (float *)mem->alloc(nBytes);
				memcpy(markerData, ptr, nBytes);
				ptr += nBytes;

				// associated marker IDs
				nBytes = nRigidMarkers*sizeof(int);
				int* markerIDs = (int*)mem->alloc(nBytes);
				memcpy(markerIDs, ptr, nBytes);
				ptr += nBytes;

				// associated marker sizes
				nBytes = nRigidMarkers*sizeof(float);
				float* markerSizes = (float*)mem->alloc(nBytes);
				memcpy(markerSizes, ptr, nBytes);
				ptr += nBytes;

				for(int k = 0; k < nRigidMarkers; k++){
					DEBUG("\tMarker %d: id=%d\tsize=%3.1f\tpos=[%3.2f,%3.2f,%3.2f]\n", k, markerIDs[k], markerSizes[k], markerData[k*3], markerData[k*3+1],markerData[k*3+2]);
				}

				// Mean marker error (2.0 and later)
				if(major >= 2){
					float fError = 0.0f; memcpy(&fError, ptr, 4); ptr += 4;
					DEBUG("Mean marker error: %3.2f\n", fError);
				}

				// Tracking flags (2.6 and later)
				if( ((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0) ){
					// params
					short params = 0; memcpy(&params, ptr, 2); ptr += 2;
					bool bTrackingValid = params & 0x01; // 0x01 : rigid body was successfully tracked in this frame
				}

			} // next rigid body

		} // next skeleton
	}

	// labeled markers (version 2.3 and later)
	if(((major == 2)&&(minor>=3)) || (major>2)){

		int nLabeledMarkers = 0;
		memcpy(&nLabeledMarkers, ptr, 4); ptr += 4;
		DEBUG("Labeled Marker Count : %d\n", nLabeledMarkers);

		frame->nLabeledMarkers = nLabeledMarkers;

		for(int j = 0; j < nLabeledMarkers; j++){

			sMarker &m = frame->LabeledMarkers[j];

			// id, x, y, z, size
			memcpy(&m, ptr, 20); ptr += 20;

			// 2.6 and later
			if(((major == 2)&&(minor >= 6)) || (major > 2) || (major == 0)){
				// marker params
				short params = 0; memcpy(&m.params, ptr, 2); ptr += 2;
				//bool bOccluded = params & 0x01;     // marker was not visible (occluded) in this frame
				//bool bPCSolved = params & 0x02;     // position provided by point cloud solve
				//bool bModelSolved = params & 0x04;  // position provided by model solve
			}

			DEBUG("ID  : %d\n", m.ID);
			DEBUG("pos : [%3.2f,%3.2f,%3.2f]\n", m.x, m.y, m.z);
			DEBUG("size: [%3.2f]\n", m.size);
		}
	}

	// Force Plate data (version 2.9 and later)
	if(((major == 2) && (minor >= 9)) || (major > 2)){
		int nForcePlates;
		memcpy(&nForcePlates, ptr, 4); ptr += 4;
		frame->nForcePlates = nForcePlates;

		for(int iForcePlate = 0; iForcePlate < nForcePlates; iForcePlate++){
			sForcePlateData &fp = frame->ForcePlates[iForcePlate];

			// ID
			int ID = 0; memcpy(&ID, ptr, 4); ptr += 4;
			DEBUG("Force Plate : %d\n", ID);
			fp.ID = ID;

			// Channel Count
			int nChannels = 0; memcpy(&nChannels, ptr, 4); ptr += 4;
			fp.nChannels = nChannels;

			// Channel Data
			for(int i = 0; i < nChannels; i++){
				sAnalogChannelData &chan = fp.ChannelData[i];

				DEBUG(" Channel %d : ", i);
				int nFrames = 0; memcpy(&nFrames, ptr, 4); ptr += 4;
				chan.nFrames = nFrames;

				for (int j = 0; j < nFrames; j++){
					float val = 0.0f;  memcpy(&val, ptr, 4); ptr += 4;
					DEBUG("%3.2f   ", val);
					chan.Values[j] = val;
				}
				DEBUG("\n");
			}
		}
	}

	// latency
	memcpy(&frame->fLatency, ptr, 4); ptr += 4;
	DEBUG("latency : %3.3f\n", frame->fLatency);

	// timecode
	memcpy(&frame->Timecode, ptr, 4); ptr += 4;
	memcpy(&frame->TimecodeSubframe, ptr, 4); ptr += 4;
//	char szTimecode[128] = "";
//	TimecodeStringify(timecode, timecodeSub, szTimecode, 128);

	// timestamp
	// 2.7 and later - increased from single to double precision
	if(((major == 2)&&(minor>=7)) || (major>2)){
		memcpy(&frame->fTimestamp, ptr, 8); ptr += 8;
	}
	else{
		float fTemp = 0.0f;
		memcpy(&fTemp, ptr, 4); ptr += 4;
		frame->fTimestamp = (double)fTemp;
	}

	// frame params
	memcpy(&frame->params, ptr, 2); ptr += 2;
	//bool bIsRecording = params & 0x01;                  // 0x01 Motive is recording
	//bool bTrackedModelsChanged = params & 0x02;         // 0x02 Actively tracked model list has changed


	// TODO: Figure out what this marker is and validate it
	// end of data tag
	int32_t eod = 0; memcpy(&eod, ptr, 4); ptr += 4;

	DEBUG("End Packet %d\n-------------\n", eod);


	// Replace and cleanup old data
	this->frame = frame;

	if(this->frameMem != NULL)
		delete this->frameMem;
	this->frameMem = mem;


	// Callback
	if(this->data_callback != NULL)
		this->data_callback(this->frame, this->data_callback_arg);

}


void ClientCore::unpackDataDescs(char *ptr){

	int major, minor;

	// number of datasets
	int nDatasets = 0; memcpy(&nDatasets, ptr, 4); ptr += 4;
	DEBUG("Dataset Count : %d\n", nDatasets);
	dataDescs->nDataDescriptions = nDatasets;

	for(int i = 0; i < nDatasets; i++){
		DEBUG("Dataset %d\n", i);
		sDataDescription &desc = dataDescs->arrDataDescriptions[i];


		int type = 0; memcpy(&type, ptr, 4); ptr += 4;
		printf("Type : %d\n", i, type);
		desc.type = type;

		if(type == Descriptor_MarkerSet){   // markerset
			// name
			char szName[256];
			strcpy(szName, ptr);
			int nDataBytes = (int) strlen(szName) + 1;
			ptr += nDataBytes;
			DEBUG("Markerset Name: %s\n", szName);

			// marker data
			int nMarkers = 0; memcpy(&nMarkers, ptr, 4); ptr += 4;
			DEBUG("Marker Count : %d\n", nMarkers);

			for(int j = 0; j < nMarkers; j++){
				char szName[256];
				strcpy(szName, ptr);
				int nDataBytes = (int) strlen(szName) + 1;
				ptr += nDataBytes;
				DEBUG("Marker Name: %s\n", szName);
			}
		}
		else if(type == Descriptor_RigidBody){   // rigid body
			if(major >= 2){
				// name
				char szName[MAX_NAMELENGTH];
				strcpy(szName, ptr);
				ptr += strlen(ptr) + 1;
				printf("Name: %s\n", szName);
			}

			int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
			DEBUG("ID : %d\n", ID);

			int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
			DEBUG("Parent ID : %d\n", parentID);

			float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
			DEBUG("X Offset : %3.2f\n", xoffset);

			float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
			DEBUG("Y Offset : %3.2f\n", yoffset);

			float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
			DEBUG("Z Offset : %3.2f\n", zoffset);

		}
		else if(type == Descriptor_Skeleton){   // skeleton
			char szName[MAX_NAMELENGTH];
			strcpy(szName, ptr);
			ptr += strlen(ptr) + 1;
			DEBUG("Name: %s\n", szName);

			int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
			DEBUG("ID : %d\n", ID);

			int nRigidBodies = 0; memcpy(&nRigidBodies, ptr, 4); ptr +=4;
			DEBUG("RigidBody (Bone) Count : %d\n", nRigidBodies);

			for(int i = 0; i < nRigidBodies; i++){
				if(major >= 2){
					// RB name
					char szName[MAX_NAMELENGTH];
					strcpy(szName, ptr);
					ptr += strlen(ptr) + 1;
					DEBUG("Rigid Body Name: %s\n", szName);
				}

				int ID = 0; memcpy(&ID, ptr, 4); ptr +=4;
				DEBUG("RigidBody ID : %d\n", ID);

				int parentID = 0; memcpy(&parentID, ptr, 4); ptr +=4;
				DEBUG("Parent ID : %d\n", parentID);

				float xoffset = 0; memcpy(&xoffset, ptr, 4); ptr +=4;
				DEBUG("X Offset : %3.2f\n", xoffset);

				float yoffset = 0; memcpy(&yoffset, ptr, 4); ptr +=4;
				DEBUG("Y Offset : %3.2f\n", yoffset);

				float zoffset = 0; memcpy(&zoffset, ptr, 4); ptr +=4;
				DEBUG("Z Offset : %3.2f\n", zoffset);
			}
		}

	}   // next dataset

	printf("End Packet\n-------------\n");

}



Memory::Memory(){

}

Memory::~Memory(){
	for(int i = 0; i < this->memory.size(); i++){
		free(this->memory[i]);
	}
}

void *Memory::alloc(size_t size){
	void *mem = malloc(size);
	this->memory.push_back(mem);
	return mem;
}
