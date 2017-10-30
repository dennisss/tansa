#include <Arduino.h>
#include <SPI.h>

#include <DW1000.h>
#include <DW1000Time.h>

#include <IRremote.h>

#include "driver/rtc_io.h"
#include "driver/gpio.h"

#include "config.h"
#include "packet.h"


enum BeaconState {
	BeaconIdle = 0, /** Just listening for packets but not actually in the middle of sending anything */
	BeaconSending = 1, /** Just sent out a packet, waiting for it to finish transmitting */
	BeaconWaiting = 2, /** A packet was sent out and we are awaiting an acknowledgement */
};

typedef void (*BeaconCallback)();

BeaconState beaconState = BeaconIdle;

BeaconCallback beaconCallback = NULL; /**< Called when the request is complete  */
bool beaconAwaitAck = false; /**< Whether or not we should wait for an ACK before calling the callback  */
uint8_t beaconSeq = 0; /**< Current sequence number. Only received ACK packets with the current sequence number will be processed */
uint8_t beaconTxBuffer[32]; BeaconPacket *beaconRxPacket;
uint8_t beaconRxBuffer[32]; BeaconPacket *beaconTxPacket;

uint8_t beaconLastRxBuffer[32];


bool beaconEventSent = false;
bool beaconEventReceived = false;

DW1000Time beaconTxTime; /**< Time at which the most recent packet was sent out */
DW1000Time beaconRxTime; /**< Time at which the most recent  */
unsigned long beaconLastPing = 0; /**< A meta field recording the last time a ping request was received */
uint32_t beaconLastActive = 0;


IRrecv irRecv(PIN_IR_RCV);
decode_results irResults;

char serialBuffer[64];
int serialBufferPosition = 0;
int serialArgc = 0;
char *serialArgv[8];


// TODO: By default enable configuration mode (if we arent )
//bool configMode = false;
bool configMode = false; // Whether or not the beacon is turns on listening for packets
//bool anchorMode = false;
bool chargingMode = false, doneCharging = false;;





void beaconDebug() {
	char msg[128];
	DW1000.getPrintableDeviceIdentifier(msg);
	Serial.print("Device ID: "); Serial.println(msg);
	DW1000.getPrintableExtendedUniqueIdentifier(msg);
	Serial.print("Unique ID: "); Serial.println(msg);
	DW1000.getPrintableNetworkIdAndShortAddress(msg);
	Serial.print("Network ID & Device Address: "); Serial.println(msg);
	DW1000.getPrintableDeviceMode(msg);
	Serial.print("Device mode: "); Serial.println(msg);
}


void beaconStart() {
	DW1000.newReceive();
	DW1000.setDefaults();
	// so we don't need to restart the receiver manually
	DW1000.receivePermanently(true);
	DW1000.startReceive();

	beaconLastActive = millis();
}

void beaconHandleSent() {
	beaconEventSent = true;
}

void beaconHandleReceived() {
	beaconEventReceived = true;
}

void beaconHandleError() {
	Serial.println("Error!");
}



// Send a message optionally at a certain time and optionally to a certain beacon
/**
 * Sends raw data over the radio
 *
 * The data should be put into the beaconTxBuffer before calling this
 *
 * @param len
 * @param callback
 *
 * @param tx_time if NULL, send immediately, otherwise, send at this time (the physical time at which it will be sent will be this + antenna delay)
 */
void beaconSend(unsigned len, BeaconCallback callback = NULL, bool awaitAck = false, const DW1000Time *tx_time = NULL) {

	// Setup state
	beaconState = BeaconSending;
	beaconAwaitAck = awaitAck;
	beaconCallback = callback;

	if(awaitAck) {
		beaconTxPacket->seq = ++beaconSeq;
	}

	beaconTxPacket->src_addr = BEACON_ID;


	// Do the transmission
	DW1000.newTransmit();
	DW1000.setDefaults();
	DW1000.setData(beaconTxBuffer, len);
	if(tx_time != NULL) {
		DW1000.setDelay(*tx_time, true);
	}
	DW1000.startTransmit();
	//delaySent = millis();
}

/**
 * Like beaconSend, but prepares the packet as a response
 */
void beaconSendResponse(unsigned len, const DW1000Time *tx_time = NULL) {
	BeaconPacket *p = beaconTxPacket;
	p->type = beaconRxPacket->type | BEACON_PACKET_ACK;
	p->seq = beaconRxPacket->seq; // TODO: Make sure that this isn't
	p->dst_addr = beaconRxPacket->src_addr;

	beaconSend(len, NULL, false, tx_time);
}


/**
 *
 */
void beaconPing(uint8_t addr, BeaconCallback callback) {
	BeaconPacket *p = beaconTxPacket;
	p->type = BEACON_PACKET_PING;
	p->dst_addr = addr;
	beaconSend(sizeof(BeaconPacket), callback, true);
}

/**
 * Respond to a ping that we just received
 */
void beaconPingRespond() {
	BeaconPacket *p = beaconTxPacket;

	// Respond exactly 1ms after it was received
	beaconTxTime = beaconRxTime + DW1000Time(1, DW1000Time::MILLISECONDS);

	beaconSendResponse(sizeof(BeaconPacket), &beaconTxTime);
}

// For the most recent ping, computes the time the signal took to get to the other beacon
void beaconPingDelta(DW1000Time *t) {
	DW1000Time deltaTime = beaconRxTime - beaconTxTime; // Raw delta

	// Pong sets a trasmit time in the future of 2ms, but that delay only starts sending at that time (need to account for delay from start to emission)
	deltaTime -= DW1000Time(1, DW1000Time::MILLISECONDS) + DW1000._antennaDelay;

	deltaTime /= 2;

	*t = deltaTime;
}


void beaconProxyPing(uint8_t src, uint8_t dst, BeaconCallback callback) {
	BeaconPacket *p = beaconTxPacket;
	p->type = BEACON_PACKET_PING;
	p->dst_addr = src;

	BeaconPacketProxyPing *pp = (BeaconPacketProxyPing *) beaconTxPacket->data;
	pp->real_dst_addr = dst;

	beaconSend(sizeof(BeaconPacket) + sizeof(BeaconPacketProxyPing), callback, true);
}


void beaconStat(uint8_t addr, BeaconCallback callback) {
	BeaconPacket *p = beaconTxPacket;
	p->type = BEACON_PACKET_STAT;
	p->dst_addr = addr;
	beaconSend(sizeof(BeaconPacket), callback, true);
}

float batteryVoltage();

void beaconStatRespond(bool send = true) {
	BeaconPacket *p = beaconTxPacket;

	BeaconPacketStatAck *s = (BeaconPacketStatAck *) p->data;

	float temp = 0, vbat = 0;
	DW1000.getTempAndVbat(temp, vbat);

	// TODO: Expand range
	vbat -= 2.8f;
	if(vbat < 0) { vbat = 0; }
	vbat = (vbat / (4.2f - 2.8f))*255;

	temp -= 10.0f;
	if(temp < 0) { temp = 0; }
	temp = (temp / 80.0f)*255;

	s->voltage_reg = vbat;
	s->temperature = temp;


	// Realistically, we only care about the battery voltage from 3V to 4.2V, so we'll scale to around that range
	float battery = batteryVoltage() - 2.8f;
	if(battery < 0) { battery = 0; }
	battery = (battery / (4.2f - 2.8f))*255;

	s->voltage_battery = (uint8_t) battery;


	if(send) {
		beaconSendResponse(sizeof(BeaconPacketStatAck));
	}
}

//
void beaconSwapBuffers() {
	for(int i = 0; i < sizeof(beaconRxBuffer); i++) {
		uint8_t temp = beaconRxBuffer[i];
		beaconRxBuffer[i] = beaconLastRxBuffer[i];
		beaconLastRxBuffer[i] = temp;
	}
}

void beaconBroadcast(const DW1000Time &tx_time) {
	BeaconPacket *p = beaconTxPacket;
	p->type = BEACON_PACKET_BROADCAST;
	p->dst_addr = BEACON_ADDR_BROADCAST;

	BeaconPacketBroadcast *b = (BeaconPacketBroadcast *) beaconTxPacket->data;
	tx_time.getTimestamp(b->time);

	// TODO: We might as well sequence these as well
	beaconSend(sizeof(BeaconPacket) + sizeof(BeaconPacketBroadcast));
}


void beaconAnchorStart() {
	/*
		Anchor Protocol:
		- First we assume that every beacon knows the exact distance to each other beacon
			- This should be converted to ideal clock ticks
		- Then we initialize each
		- At the very beginning, ever anchor sits waiting for a message from another beacon

		- The computer triggers one anchor to send a start pulse
			- The time at which it transmits will be defined as global time 0

		- Once the second anchor receives this time, it will send it's send it's pulse at:
			- "Trecv0 - ToF_0_1 + (1/N)" where N is the total number of beacons
			- Then it will schedule it's next

		- Third anchor receives both


		- After many cycles,
			- Anchor considers last N-1 times received


		- Basically, whenever an anchor receives any another anchor's broadcast, it is allowed to

		The objective of the clock synchronization is to estimate the hidden ideal clock model

	*/

}





void beaconReceiver() {
/*
	DW1000.getData(message);
    Serial.print("Received message ... #"); Serial.println(numReceived);
    Serial.print("Data is ... "); Serial.println(message);
    Serial.print("FP power is [dBm] ... "); Serial.println(DW1000.getFirstPathPower());
    Serial.print("RX power is [dBm] ... "); Serial.println(DW1000.getReceivePower());
    Serial.print("Signal quality is ... "); Serial.println(DW1000.getReceiveQuality());
    received = false;
*/
}

void beaconInit() {

	beaconRxPacket = (BeaconPacket *) beaconRxBuffer;
	beaconTxPacket = (BeaconPacket *) beaconTxBuffer;

	// Initialize DW1000 (the .begin will also wakeup the device if it was previously asleep)
	DW1000.begin(PIN_DW_IRQ, PIN_DW_RST, PIN_DW_WAKEUP);
	DW1000.select(PIN_DW_SS);
	Serial.println(F("DW1000 initialized ..."));

	DW1000.newConfiguration();
	DW1000.setDefaults();
	DW1000.setDeviceAddress(6);
	DW1000.setNetworkId(10);
	DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY); // MODE_LONGDATA_RANGE_LOWPOWER
	DW1000.interruptOnSent(true);
	DW1000.interruptOnReceived(true);
	DW1000.interruptOnReceiveFailed(true);
	DW1000.interruptOnReceiveTimeout(true);
	DW1000.commitConfiguration();
	Serial.println(F("Committed configuration ..."));

	//beaconDebug();

	DW1000.attachSentHandler(beaconHandleSent);
	DW1000.attachReceivedHandler(beaconHandleReceived);
	DW1000.attachReceiveFailedHandler(beaconHandleError);
	DW1000.attachErrorHandler(beaconHandleError);
}

void beaconProxyPingCallback() {
	BeaconPacketProxyPingAck *p = (BeaconPacketProxyPingAck *) beaconTxPacket->data;
	DW1000Time deltaTime;
	beaconPingDelta(&deltaTime);

	beaconSwapBuffers();
	beaconSendResponse(sizeof(BeaconPacket) + sizeof(BeaconPacketProxyPingAck));
}

/**
 * Called whenever a new packet was received
 */
void beaconHandlePacket() {

	if(beaconRxPacket->type == BEACON_PACKET_PING) {
		beaconLastPing = millis();
		beaconPingRespond();
	}
	else if(beaconRxPacket->type == BEACON_PACKET_STAT) {
		beaconStatRespond();
	}
	else if(beaconRxPacket->type == BEACON_PACKET_PROXY_PING) {
		BeaconPacketProxyPing *p = (BeaconPacketProxyPing *) beaconRxPacket->data;
		uint8_t addr = p->real_dst_addr;

		beaconSwapBuffers(); // Save the proxy request
		beaconPing(addr, beaconProxyPingCallback);
	}

}


/**
 * Runs a
 */
void beaconCycle() {
	int32_t curMillis = millis();
	if(!beaconEventSent && !beaconEventReceived) {
		// check if inactive
		if(curMillis - beaconLastActive > 250) {
			beaconStart();
		}
		return;
	}


	if(beaconEventSent) {
		beaconLastActive = curMillis;
		beaconEventSent = false;
		DW1000.getTransmitTimestamp(beaconTxTime);

		if(beaconAwaitAck) {
			beaconState = BeaconWaiting;
		}
		else {
			beaconState = BeaconIdle;
			if(beaconCallback != NULL) {
				beaconCallback();
				beaconCallback = NULL;
			}
		}
	}
	else if(beaconEventReceived) {

		// TODO: Assert beaconState == Beachon ||

		beaconLastActive = curMillis;
		beaconEventReceived = false;
		int len = DW1000.getDataLength();
		DW1000.getData(beaconRxBuffer, len);
		DW1000.getReceiveTimestamp(beaconRxTime);

		//
		if(beaconRxPacket->dst_addr != BEACON_ID && beaconRxPacket->dst_addr != BEACON_ADDR_BROADCAST) {
			return;
		}

		if(beaconRxPacket->type & BEACON_PACKET_ACK) {
			if(beaconRxPacket->seq == beaconSeq && beaconAwaitAck && beaconCallback != NULL) {
				beaconState = BeaconIdle;
				beaconCallback();
				beaconCallback = NULL;
			}
		}
		else {
			// handle unknown inbound packet
			beaconHandlePacket();
		}
	}
	else if(beaconState != BeaconIdle) {

		// We may have to resend it

	}
}


bool isUsbConnected();

void goToSleep() {
	// See https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/DeepSleep/ExternalWakeUp/ExternalWakeUp.ino

	DW1000.deepSleep(false, true);

	// Keep RTC IO turned on for using push-pull resistors
	esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
	rtc_gpio_pulldown_en((gpio_num_t) PIN_DW_WAKEUP);

	// Configure wakeup triggers
	if(!isUsbConnected()) {
		esp_deep_sleep_enable_ext0_wakeup((gpio_num_t) PIN_VUSB, 1);
	}
	esp_deep_sleep_enable_ext1_wakeup(1 << PIN_IR_RCV, ESP_EXT1_WAKEUP_ALL_LOW);

	esp_deep_sleep_start();
}


bool isUsbConnected() {
	return digitalRead(PIN_VUSB) == HIGH;
}

float batteryVoltage() {
	float v = 3.30f * (127.0f/100.0f) * float(analogRead(PIN_VBAT_SCAL)) / 4096.0f;  // LiPo battery
	return v;
}

bool isCharging() {
	float v = batteryVoltage();
	return isUsbConnected() && (v > 0.2);
}

bool isChargingDone() {
	return false; // Voltage currently only accurate while not chargin
	return batteryVoltage() > 4.1;
}

void remoteCycle() {
	// Handle a remote control code
	if(!irRecv.decode(&irResults)) {
		return;
	}

	uint32_t v = irResults.value;

	//Serial.print("Code: ");
	//Serial.println(v, HEX);
	v &= 0x0000ffff;


	if((irResults.value & 0xffff0000) != CODE_MASK) {
		// Unknown code group
		goto remoteCycleExit;
	}

	if(v == CODE_POWER) {
		if(!configMode) {
			beaconStart();
			configMode = true;
		}

		goto remoteCycleExit;
	}

	// Must be turned on to do anything else
	if(!configMode) {
		goto remoteCycleExit;
	}

	if(v == CODE_B) { // Turn all off
		goToSleep();
	}
	// TODO:

remoteCycleExit:
	irRecv.resume(); // Receive the next value
}




void serialPrintDeltaTime(DW1000Time &deltaTime) {
	int64_t ticks = deltaTime.getTimestamp();

	float meters = deltaTime.getAsMeters();

	Serial.print(BEACON_ERROR_OK);
	Serial.print(" ");
	Serial.print((int) ticks);
	Serial.print(" ");
	Serial.println(meters, 4);
}

void serialPingCallback() {
	DW1000Time deltaTime;
	beaconPingDelta(&deltaTime);

	serialPrintDeltaTime(deltaTime);
}

void serialProxyPingCallback() {
	BeaconPacketProxyPingAck *pp = (BeaconPacketProxyPingAck *) beaconRxPacket->data;

	DW1000Time deltaTime;
	deltaTime.setTimestamp(pp->delta_time);

	serialPrintDeltaTime(deltaTime);
}



void serialStatPrint(BeaconPacketStatAck *s) {

	float voltage_reg = (4.2 - 2.8)*(s->voltage_reg / 255.0f) + 2.8;
	float voltage_battery = (4.2 - 2.8)*(s->voltage_battery / 255.0f) + 2.8;
	float temp = 80.0*(s->temperature / 255.0f) + 10.0;

	Serial.print(BEACON_ERROR_OK);
	Serial.print("  VBat: ");
	Serial.print(voltage_battery, 2);
	Serial.print("  VReg: ");
	Serial.print(voltage_reg, 2);
	Serial.print("  Temp: ");
	Serial.println(temp, 2);
}

void serialStatCallback() {
	BeaconPacketStatAck *s = (BeaconPacketStatAck *) beaconRxPacket->data;
	serialStatPrint(s);
}




void serialHandleCommand() {
	int argc = serialArgc;
	char **argv = serialArgv;

	char *func = argv[0];

	if(strcmp(func, "ping") == 0) {
		if(argc != 2 && argc != 3) {
			Serial.print(BEACON_ERROR_USAGE);
			Serial.println(" Usage: ping [id] [id2]");
			return;
		}

		int id1 = atoi(argv[1]);
		if(argc == 2) {
			beaconPing(id1, serialPingCallback);
		}
		else if(argc == 3) {
			int id2 = atoi(argv[2]);
			beaconProxyPing(id1, id2, serialProxyPingCallback);
		}
	}
	else if(strcmp(func, "stat") == 0) {
		if(argc != 1 && argc != 2) {
			Serial.print(BEACON_ERROR_USAGE);
			Serial.println(" Usage: stat [id]");
			return;
		}

		if(argc == 1) { // Stat self
			beaconStatRespond(false);
			serialStatPrint((BeaconPacketStatAck *) beaconTxPacket->data);
		}
		else {
			int id = atoi(argv[1]);
			beaconStat(id, serialStatCallback);
		}

	}

}

void serialCycle() {
	if(Serial.available() <= 0) {
		return;
	}

	char c = Serial.read();
	if(c == '\n') {
		c = '\0';
	}

	serialBuffer[serialBufferPosition++] = c;

	// Overflow
	if(serialBufferPosition >= sizeof(serialBuffer)) {
		serialBufferPosition = 0;
		return;
	}


	if(c != '\0') {
		return;
	}

	Serial.print("> ");
	Serial.println(serialBuffer);

	// Otherwise, we got an entire line

	serialArgc = 0;
	serialBufferPosition = 0;

	char *arg = strtok(serialBuffer, " ");
	while(arg != NULL) {
		serialArgv[serialArgc++] = arg;

		// Overflow max number of args
		if(serialArgc >= 8) {
			return;
		}

		arg = strtok(NULL, " ");
	}

	if(serialArgc > 0) {
		serialHandleCommand();
	}

}



// the setup function runs once when you press reset or power the board
void setup() {
	// Initialize pins
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_VBAT_SCAL, INPUT);
	pinMode(PIN_VUSB, INPUT);
	Serial.begin(115200);
	delay(10);

	// See https://github.com/espressif/esp-idf/blob/master/examples/peripherals/rmt_nec_tx_rx/main/infrared_nec_main.c for how to do this natively
	// Initialize the IR receiver
	irRecv.enableIRIn(); // Start the receiver

	beaconInit();
	// TODO: Use getTempAndVbat for calibration purposes

	unsigned long startTime = millis();

	int iter = 0;
	while(true) {

		// Update inputs
		if(iter % 10000 == 0) {
			chargingMode = isCharging();
		}


		remoteCycle();

		if(configMode) {
			beaconCycle();
		}

		if(chargingMode) { // Only need to check serial if the usb is plugged in
			serialCycle();
		}


		// Operate based on current mode
		uint8_t led_pattern = 0;
		if(configMode) {
			led_pattern = LED_ANCHOR;
		}
		else if(chargingMode) { // Charging mode takes lowest priority
			led_pattern = doneCharging? LED_CHARGED : LED_CHARGING;
		}
		else {
			led_pattern = LED_INIT;

			// If we haven't locked into any configuration since we've started, we should go back to sleep
			// Most likely this is the first boot or there was a false interrupt
			if(millis() - startTime > 1000) { // TODO: Check for overflow?
				break;
			}
		}


		// Update the LED
		if(iter % 1000 == 0) {
			uint8_t led_phase = (millis() % 1000L) / (1000L / NLED_PHASES);

			int value = (led_pattern >> (7 - led_phase)) & 1;

			if(millis() - beaconLastPing < 250) {
				value = 0;
			}


			digitalWrite(PIN_LED, value);
		}

		//Serial.println(millis());

		//delay(20);
		iter++;
	}

	goToSleep();
	return;
}


/*
void getUniqueId() {
	// See https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/ChipID/GetChipID/GetChipID.ino

	// The chip id will be used to uniquely identify this beacon
	chipid = ESP.getEfuseMac();//The chip ID is essentially its MAC address(length: 6 bytes).
	Serial.printf("ESP32 Chip ID = %04X",(uint16_t)(chipid>>32));//print High 2 bytes
	Serial.printf("%08X\n",(uint32_t)chipid);//print Low 4bytes.
}
*/

// the loop function runs over and over again forever
void loop() {
	// We won't use this
}
