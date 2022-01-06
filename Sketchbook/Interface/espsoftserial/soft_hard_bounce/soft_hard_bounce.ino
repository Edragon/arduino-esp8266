// On ESP8266:
// At 80MHz runs up 57600ps, and at 160MHz CPU frequency up to 115200bps with only negligible errors.
// Connect pin 13 to 15.
// For verification and as a example for how to use SW serial on the USB to PC connection,
// which allows the use of HW Serial on GPIO13 and GPIO15 instead, #define SWAPSERIAL below.
// Notice how the bitrates are also swapped then between RX/TX and GPIO13/GPIO15.
// Builtin debug output etc. must be stopped on HW Serial in this case, as it would interfere with the
// external communication on GPIO13/GPIO15.

#include <SoftwareSerial.h>


SoftwareSerial soft_Serial;


void setup() {
    Serial.begin(115200);
    // Important: the buffer size optimizations here, in particular the isrBufSize (11) that is only sufficiently
    // large to hold a single word (up to start - 8 data - parity - stop), are on the basis that any char written
    // to the loopback SoftwareSerial adapter gets read before another write is performed.
    // Block writes with a size greater than 1 would usually fail. Do not copy this into your own project without
    // reading the documentation.
    soft_Serial.begin(57600, SWSERIAL_8N1, 13, 15, false, 95, 11);

    Serial.println(PSTR("\nSoftware serial test started"));

    for (char ch = ' '; ch <= 'z'; ch++) {
        soft_Serial.write(ch);
    }
    soft_Serial.println();
}

void loop() {
    while (soft_Serial.available() > 0) {
        Serial.write(soft_Serial.read());
        yield();
    }
    while (Serial.available() > 0) {
        soft_Serial.write(Serial.read());
        yield();
    }
}
