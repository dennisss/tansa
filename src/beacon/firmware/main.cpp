#include <Arduino.h>

#include <SPI.h>

#include <DW1000.h>
#include <DW1000Time.h>

#include <IRremote.h>

#include <WiFi.h>


#include "driver/rtc_io.h"
#include "driver/gpio.h"


// NOTE: On the ESP32 Thing, VUSB is pulled down by a 10K resistor

// Used to sense when the battery is charging (this is active high)
// on the Sparkfun board, VUSB is conveniently pulled down by a 10k so we don't need any internal pulls on this
const uint8_t PIN_VBAT_SCAL = 12; // V = 0.787*Vbat  +/- 1%  (V = 0 when no battery is connected)
const uint8_t PIN_LED = 15;
const uint8_t PIN_VUSB = 2; // High if connected
const uint8_t PIN_IR_RCV = 4; // (this is active low and is pulled up in the receiver)
const uint8_t PIN_DW_WAKEUP = 25;
const uint8_t PIN_DW_RST = 26; // reset pin
const uint8_t PIN_DW_IRQ = 27; // irq pin
const uint8_t PIN_DW_SS = 5; // spi select pin

// Based on this remote: https://learn.sparkfun.com/tutorials/ir-control-kit-hookup-guide?_ga=2.140662258.1359352254.1508469480-972263269.1469291776
const uint32_t CODE_MASK = 0x10ef0000;
const uint32_t CODE_POWER = 0xd827; // PWR button, turns on configuration mode
const uint32_t CODE_A = 0xf807; // Turns on anchor mode
const uint32_t CODE_B = 0x7887; // Turns off everything
const uint32_t CODE_C = 0x58a7; // Turns on configuration/tag mode
const uint32_t CODE_CENTER = 0x20df;
const uint32_t CODE_LEFT = 0x10ef;
const uint32_t CODE_UP = 0xa05f;
const uint32_t CODE_RIGHT = 0x807f;
const uint32_t CODE_DOWN = 0x00ff;

const uint8_t NLED_PHASES = 8; // Number of LED phases per second (where each phase is a distinct high low value)
const uint8_t LED_CHARGING = 0b11110000;
const uint8_t LED_CONFIGURING = 0b1001000;
const uint8_t LED_CHARGED = 0b11111111;
const uint8_t LED_ANCHOR = 0b11111111;



const char *WIFI_SSID = "yourssid";
const char *WIFI_PASS = "yourpasswd";

// Enable this whenever using this in public
// This will prevent the controller from exiting anchor mode via an IR remote command
const bool ANCHOR_LOCK = false;


WiFiServer server(80);

IRrecv irRecv(PIN_IR_RCV);
decode_results irResults;



bool isUsbConnected() {
	return digitalRead(PIN_VUSB) == HIGH;
}

float batteryVoltage() {
	return 3.30f * (127.0f/100.0f) * float(analogRead(34)) / 4096.0f;  // LiPo battery
    //Serial.print("Battery Voltage = "); Serial.print(VBAT, 2); Serial.println(" V");
}

bool isCharging() {
	float v = batteryVoltage();
	return isUsbConnected() && v > 0.2;
}

bool isChargingDone() {
	return batteryVoltage() > 4.1;
}

void wifiStart() {
	Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

	WiFi.setHostname("tansa-beacon-1");
	// TODO: Also configure a
	WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);


    while(WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

	server.begin();


}

void wifiStop() {
	server.end();
	WiFi.setAutoReconnect(false);
	WiFi.disconnect(true);
}

void wifiPoll() {
	WiFiClient client = server.available();   // listen for incoming clients

	if(client) {                             // if you get a client,
		Serial.println("new client");           // print a message out the serial port
		String currentLine = "";                // make a String to hold incoming data from the client
		while(client.connected()) {            // loop while the client's connected
			if (client.available()) {             // if there's bytes to read from the client,
				char c = client.read();             // read a byte, then

			Serial.write(c);                    // print it out the serial monitor
			if(c == '\n') {                    // if the byte is a newline character

				// if the current line is blank, you got two newline characters in a row.
				// that's the end of the client HTTP request, so send a response:
				if (currentLine.length() == 0) {
					// HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
					// and a content-type so the client knows what's coming, then a blank line:
					client.println("HTTP/1.1 200 OK");
					client.println("Content-type:text/html");
					client.println();

					// the content of the HTTP response follows the header:
					client.print("Click <a href=\"/H\">here</a> turn the LED on pin 5 on<br>");
					client.print("Click <a href=\"/L\">here</a> turn the LED on pin 5 off<br>");

					// The HTTP response ends with another blank line:
					client.println();
					// break out of the while loop:
					break;
				} else {    // if you got a newline, then clear currentLine:
					currentLine = "";
				}
			} else if (c != '\r') {  // if you got anything else but a carriage return character,
				currentLine += c;      // add it to the end of the currentLine
			}

			// Check to see if the client request was "GET /H" or "GET /L":
			if (currentLine.endsWith("GET /H")) {
				digitalWrite(5, HIGH);               // GET /H turns the LED on
			}
			if (currentLine.endsWith("GET /L")) {
				digitalWrite(5, LOW);                // GET /L turns the LED off
			}
		}
	}
	// close the connection:
	client.stop();
	Serial.println("client disonnected");
}
}



void goToSleep() {
	// See https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/DeepSleep/ExternalWakeUp/ExternalWakeUp.ino

	DW1000.deepSleep(false, true);

	// Keep RTC IO turned on for using push-pull resistors
	esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
	rtc_gpio_pulldown_en((gpio_num_t) PIN_DW_WAKEUP);

	// Configure wakeup triggers
	esp_deep_sleep_enable_ext0_wakeup((gpio_num_t) PIN_VUSB, 1);
	esp_deep_sleep_enable_ext1_wakeup(1 << PIN_IR_RCV, ESP_EXT1_WAKEUP_ALL_LOW);

	esp_deep_sleep_start();
}


// the setup function runs once when you press reset or power the board
void setup() {
	// Initialize pins
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_VBAT_SCAL, INPUT);
	Serial.begin(9600);

	// Initialize the IR receiver
	irRecv.enableIRIn(); // Start the receiver

	// Initialize DW1000 (the .begin will also wakeup the device if it was previously asleep)
	DW1000.begin(PIN_DW_IRQ, PIN_DW_RST, PIN_DW_WAKEUP);
	DW1000.select(PIN_DW_SS);
	Serial.println(F("DW1000 initialized ..."));

	DW1000.newConfiguration();
	DW1000.enableMode(DW1000.MODE_SHORTDATA_FAST_ACCURACY); // MODE_LONGDATA_RANGE_LOWPOWER

	DW1000.setNetworkId(10);
	DW1000.setDeviceAddress(5);
	DW1000.commitConfiguration();
	Serial.println(F("Committed configuration ..."));

	// TODO: Use getTempAndVbat for calibration purposes

	// TODO: By default enable configuration mode (if we arent )
	bool configMode = false;
	bool anchorMode = false;
	bool chargingMode = false;

	unsigned long startTime = millis();

	while(true) {

		// Update inputs
		chargingMode = isCharging();

		// Handle a remote control code
		if(irRecv.decode(&irResults) && (irResults.value & 0xffff0000 == CODE_MASK)) {
			uint32_t v = irResults.value;
			v &= 0xffff0000;

			Serial.println(v, HEX);

			if(v == CODE_POWER || v == CODE_C) { // Power on and start config mode
				if(anchorMode) {
					// stop anchor mode
				}

				if(!configMode) {
					wifiStart();
				}

				anchorMode = false;
				configMode = true;
			}
			else if(v == CODE_B) { // Turn all off
				if(anchorMode) {
					// stop
				}
				if(configMode) {
					wifiStop();
				}
			}
			else if(v == CODE_A) { // Anchor mode
				if(configMode) {
					wifiStop();
				}
				if(!anchorMode) {
					// start
				}

				anchorMode = true;
				configMode = false;
			}

			irRecv.resume(); // Receive the next value
		}


		// Operate based on current mode
		uint8_t led_pattern = 0;
		if(anchorMode) {
			led_pattern = LED_ANCHOR;
		}
		else if(configMode) {
			led_pattern = LED_ANCHOR;
			wifiPoll();
		}
		else if(chargingMode) { // Charging mode takes lowest priority
			led_pattern = isChargingDone()? LED_CHARGED : LED_CHARGING;
		}
		else {
			// If we haven't locked into any configuration since we've started, we should go back to sleep
			// Most likely this is the first boot or there was a false interrupt
			if(millis() - startTime > 200) { // TODO: Check for overflow?
				break;
			}
		}

		// Update the LED
		uint8_t led_phase = (millis() % 1000L) / NLED_PHASES;
		digitalWrite(PIN_LED, (led_pattern >> (7 - led_pattern)) & 1);

		delay(5);
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
