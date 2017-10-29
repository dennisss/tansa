

/**
 * This ranges from 1-15 and a unique number should be programmed to every distinct anchor device
 * This is used as the address of this beacon
 */
const uint8_t BEACON_ID = 1;

// Enable this whenever using this in public
// This will prevent the controller from exiting anchor mode via an IR remote command
const bool ANCHOR_LOCK = false;



// NOTE: On the ESP32 Thing, VUSB is pulled down by a 10K resistor

// Used to sense when the battery is charging (this is active high)
// on the Sparkfun board, VUSB is conveniently pulled down by a 10k so we don't need any internal pulls on this
const uint8_t PIN_VBAT_SCAL = 14; // V = 0.787*Vbat  +/- 1%  (V = 0 when no battery is connected)
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
const uint8_t LED_CHARGING = 0b11000000;
const uint8_t LED_CONFIGURING = 0b1001000;
const uint8_t LED_CHARGED = 0b11111111;
const uint8_t LED_ANCHOR = 0b11111111;
const uint8_t LED_INIT = 0b10101010;
