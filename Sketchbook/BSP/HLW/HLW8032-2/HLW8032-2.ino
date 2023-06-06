// On ESP8266:
// At 80MHz runs up 57600ps, and at 160MHz CPU frequency up to 115200bps with only negligible errors.
// Connect pin 12 to 14.

#include <SoftwareSerial.h>

#define BAUD_RATE 4800
SoftwareSerial swSer;

char buf[24];

void setup() {
  Serial.begin(BAUD_RATE);
  swSer.begin(BAUD_RATE, SWSERIAL_8N1, 4, 5, false, 95, 11);


}

void loop() {
  const uint8_t PACKET_START = 242;

  //read the serial buffer
  while ( Serial.available() ) {
    char ch = Serial.read();
    // collectPacket = false
    if (ch == PACKET_START) {
      int rlen = Serial.readBytes(buf, 24);

      // prints the received data
      Serial.print("I received: ");
      for (int i = 0; i < rlen; i++) {
        Serial.print(buf[i]);
      }
    }
  }
}
