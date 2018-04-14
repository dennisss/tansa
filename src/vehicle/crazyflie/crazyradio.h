#ifndef CRAZYRADIO_H
#define CRAZYRADIO_H

/*
	Crazyradio USB driver
	By itself it does not buffer messages in either direction.

	- To use it you need to specify a libusb context, a message handler, and a message emitter
	- Also, in the main loop of the program, libusb needs to be polled
*/

#include <stdint.h>
#include <libusb.h>

#include <vector>
#include <memory>
#include <string>

#include <crtp.h>


#define CRAZYRADIO_PA_VID 0x1915
#define CRAZYRADIO_PA_PID 0x7777


/**
 * Describes a single configuration state of the radio. Each Crazyflie will requires a unique configuration to be matched by the radio for it to receive data.
 */
struct RadioUri {
	int num; /**< The number of this radio. Should be unique per radio */
	uint8_t channel; /**< Desired channel  */
	uint8_t rate; /**< Desired rate */
	uint64_t addr; /**< Desired address (only the 5 least significant bytes of this are used). Set to 0 to broadcast */
};


/**
 * Called by the radio to get a new message to send
 *
 * @param uri the current radio configuration. this should be modified to match which client should recieve the next message
 * @param buf if a message is available, the function should fill this with the contents
 * @param arg a user specified argument that was given to set_callbacks
 * @return 0 if no message is available otherwise 1
 */
typedef int (*CrazyradioFetcher)(RadioUri *uri, crtp_message_t *buf, void *arg);

/**
 * Called by the radio with each message recieved from the radio
 *
 * @param status
 * @param uri the configuration on which a message was received
 * @param msg the raw message that was received
 * @param arg
 * @return whether or not we should
 */
typedef int (*CrazyradioHandler)(int status, RadioUri *uri, crtp_message_t *msg, void *arg);


typedef enum {
	CFRADIO_STATE_IDLE = 0, /* doing nothing */
	CFRADIO_STATE_CONFIGURING, /* in the process of reconfiguring rx/tx parameters */
	CFRADIO_STATE_SENDING, /* sending a data message */
	CFRADIO_STATE_RECEIVING /* receiving a data message */
} cfradio_state;

/**
 * Used internally. Do not use! Format of messages from the Crazyradio
 */
typedef struct {
	uint8_t status;
	char data[32];
} usb_message;



class Crazyradio {
public:

	typedef std::shared_ptr<Crazyradio> Ptr;

	static bool Parse(std::string uri, RadioUri &out);

	/**
	 * Finds all connected Crazyradios
	 */
	static std::vector<Crazyradio::Ptr> Enumerate(libusb_context *ctx);


	/**
	 * Creates a new Crazyradio
	 *
	 * @param device a usb device
	 * @param num a unique number identifying this radio. the radio will only operate on uris with this number
	 */
	Crazyradio(libusb_device *device, int num);
	~Crazyradio();


	int open();
	int open(libusb_device_handle *handle);
	int close();

	/**
	 * Completely reconfigures the radio for the given settings
	 */
	int set_config(const RadioUri &uri);

	/**
	 * Call when you want the radio to immediately start doing something
	 * This internally calls the fetcher. No matter what, this triggers some type of message to be sent
	 * This should be called periodically by the driver to keep polling for messages to be received
	 *
	 * Behavior: If the fetcher has an available message, then that is sent. Otherwise a null message is sent  to allow the Crazyflie to send things back
	 */
	int notify();


	void set_callbacks(CrazyradioFetcher fetcher, CrazyradioHandler handler, void *arg);


	int get_number() { return this->number; }

private:

	friend void crazyradio_transfer_callback(struct libusb_transfer *transfer);


	int submit_transfer(int dir);
	int submit_null();

	/**
	 * should be between 0 and  125
	 */
	int set_radio_channel(uint8_t channel);
	int set_radio_address(uint64_t addr, bool async = false);
	int set_data_rate(uint8_t rate);

	/**
	 * 0  -18dBm
	 * 1  -12dBm
	 * 2  -6dBm
	 * 3  0dBm
	 */
	int set_radio_power(int level);

	/*
	Value	ARD wait time
	0x00	250us
	0x01	500us
	…	…
	0x0F	4000us

	Value	ACK payload length
	0x80	0Byte
	0x81	1Byte
	…	…
	0xA0	32Bytes
	*/
	int set_radio_ard(uint8_t length);

	/**
	 * Number of retries (default to 3)
	 */
	int set_radio_arc(uint8_t nretries);

	/**
	 * 0 Auto ACK deactivated
	 * Not 0 Auto ACK enable (default)
	 */
	int set_ack_enable(bool enabled);
	int set_cont_carrier(bool active);


	libusb_device *device;
	libusb_device_handle *handle;
	cfradio_state state;

	RadioUri active_config;
	int number;

	struct libusb_transfer *transfer;
	usb_message inbuf;
	crtp_message_t outbuf; bool outvalid;
	unsigned char cfgbuf[16];

	int connected;

	int success_count, fail_count;

	//
	CrazyradioFetcher fetcher;
	CrazyradioHandler handler;
	void *arg;

};

// Used internally. Don't use directly
void crazyradio_transfer_callback(struct libusb_transfer *transfer);

#endif
