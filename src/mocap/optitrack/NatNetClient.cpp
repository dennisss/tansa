#include "NatNetClient.h"

#include <stdio.h>
#include <string.h>

//#include "ClientCore.h"
//#include "NatNetTypes.h"

#define DEFAULT_MULTICAST_ADDRESS		"239.255.42.99"     // IANA, local network
#define DEFAULT_PORT_COMMAND            1510
#define DEFAULT_PORT_DATA  			    1511




/**
 * Creates a new (multicast) instance of a NatNet Client.
 */
NatNetClient::NatNetClient() : NatNetClient(ConnectionType_Multicast){}

/**
 * Creates a new instance of a NatNet Client using the specified connection protocol.
 */
NatNetClient::NatNetClient(int iType){
	this->m_pClientCore = new ClientCore(iType);
	this->m_pClientCore->setMulticastAddress(DEFAULT_MULTICAST_ADDRESS);
	this->m_iConnectionType = iType;
}


NatNetClient::~NatNetClient(){

	delete this->m_pClientCore;
}

int NatNetClient::Initialize(const char *szLocalAddress, const char *szServerAddress){
	return this->Initialize(szLocalAddress, szServerAddress, DEFAULT_PORT_COMMAND);
}
int NatNetClient::Initialize(const char *szLocalAddress, const char *szServerAddress, int HostCommandPort) {
	return this->Initialize(szLocalAddress, szServerAddress, HostCommandPort, DEFAULT_PORT_DATA);
}


/**
 * Initializes client socket and attempts to connect to a NatNet server at the specified address.
 *
 * @param szLocalAddress IP address of client
 * @param szServerAddress IP address of server
 * @param HostCommandPort server command port (default = 1510)
 * @param HostDataPort server data port (default = 1511)
 * @return 0 if successful, error code otherwise
 */
int NatNetClient::Initialize(const char *szLocalAddress, const char *szServerAddress, int HostCommandPort, int HostDataPort){

	this->m_pClientCore->start(szLocalAddress, szServerAddress, HostCommandPort, HostDataPort);
	return 0;
}

/**
 * Disconnects from the current NatNet Server.
 *
 * @return 0 if successful, error code otherwise.
 */
int NatNetClient::Uninitialize(){
	this->m_pClientCore->stop();
	return 0;
}

/**
 * Retrieves the version of the NatNet library the client is using.
 *
 * @param Version version array (form: major.minor.build.revision)
 */
void NatNetClient::NatNetVersion(unsigned char Version[4]){

	Version[0] = 2;
	Version[1] = 9;
	Version[2] = 0;
	Version[3] = 0;
}

/**
 * Sets the message reporting level for internal NatNet messages.
 *
 * @param iLevel Verbosity level (see Verbosity level in NatNetTypes.h)
 */
void NatNetClient::SetVerbosityLevel(int level){

}

int NatNetClient::SetDataCallback(void (*CallbackFunction)(sFrameOfMocapData* pFrameOfData, void* pUserData), void* pUserData){
	this->m_pClientCore->data_callback = CallbackFunction;
	this->m_pClientCore->data_callback_arg = pUserData;
	return 0;
}

int NatNetClient::SetMessageCallback(void (*CallbackFunction)(int id, char *szTraceMessage)){



	return 0;
}


void NatNetClient::SendMessage(char *szMessage){
	sPacket PacketOut;

	PacketOut.iMessage = NAT_MESSAGESTRING; // TODO: Should this be ?
	PacketOut.nDataBytes = (int) strlen(szMessage) + 1;
	strcpy(PacketOut.Data.szData, szMessage);
	this->m_pClientCore->sendPacket(&PacketOut);
}

void NatNetClient::SendMessage1(char* szMessage){

}


int NatNetClient::SendMessageAndWait(char* szMessage, void** ppServerResponse, int* pResponseSize){

	return 0;
}

int NatNetClient::SendMessageAndWait(char* szMessage, int tries, int timeout, void** ppServerResponse, int* pResponseSize){

	return 0;
}

/**
 * Requests a description of the current NatNet server the client is connected to. This call blocks until request is responded to or times out.
 *
 * @param pServerDescription Description of the NatNet server
 * @return On success, number of data objects. 0 otherwise.
 */
int NatNetClient::GetServerDescription(sServerDescription *pServerDescription){

	return 0;
}

/**
 * Requests a description of the current streamed data objects from the server app. This call blocks until request is responded to or times out.
 *
 * @param pDataDescriptions Array of Data Descriptions.
 * @return On success, number of data objects. 0 otherwise.
 */
int NatNetClient::GetDataDescriptions(sDataDescriptions** pDataDescriptions){

	return 0;
}

/**
 * Retrieves the most recently received frame of mocap data.
 *
 * @return Frame of Mocap Data
 */
sFrameOfMocapData* NatNetClient::GetLastFrameOfData(){
	return this->m_pClientCore->frame; // TODO: This is not threadsafe
}


/**
 * Sets the NatNet server multicast group/address to connect to. SetMulticastAddress() must be called before calling Initialize(...).
 *
 * @param szCommand application defined Message string
 */
void NatNetClient::SetMulticastAddress(const char *szMulticast){
	this->m_pClientCore->setMulticastAddress(szMulticast);
}





bool NatNetClient::DecodeTimecode(unsigned int inTimecode, unsigned int inTimecodeSubframe, int* hour, int* minute, int* second, int* frame, int* subframe){
	bool bValid = true;

	*hour = (inTimecode>>24)&255;
	*minute = (inTimecode>>16)&255;
	*second = (inTimecode>>8)&255;
	*frame = inTimecode&255;
	*subframe = inTimecodeSubframe;

	return bValid;
}

bool NatNetClient::TimecodeStringify(unsigned int inTimecode, unsigned int inTimecodeSubframe, char *Buffer, int BufferSize){
	bool bValid;
	int hour, minute, second, frame, subframe;
	bValid = DecodeTimecode(inTimecode, inTimecodeSubframe, &hour, &minute, &second, &frame, &subframe);

	snprintf(Buffer, BufferSize, "%2d:%2d:%2d:%2d.%d", hour, minute, second, frame, subframe);
	for(unsigned int i = 0; i < strlen(Buffer); i++){
		if(Buffer[i]==' ')
			Buffer[i]='0';
	}

	return bValid;
}
