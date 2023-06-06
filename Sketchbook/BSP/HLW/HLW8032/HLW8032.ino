// On ESP8266:
// At 80MHz runs up 57600ps, and at 160MHz CPU frequency up to 115200bps with only negligible errors.
// Connect pin 12 to 14.

#include <SoftwareSerial.h>

#define BAUD_RATE 4800
SoftwareSerial swSer;

void setup() {
  Serial.begin(BAUD_RATE);
  swSer.begin(BAUD_RATE, SWSERIAL_8N1, 4, 5, false, 95, 11);


}

void loop() {
  const uint8_t PACKET_START = 242;
  const uint8_t PACKET_LENGTH = 24;

  boolean collectPacket = false;
  boolean packetAvailable = false;

  uint8_t packetLength = 0;
  uint8_t packet[PACKET_LENGTH + 1];

  //read the serial buffer
  while ( Serial.available() ) {
    char ch = Serial.read();
    if (!collectPacket) {                   //start of packet
      collectPacket = (ch == PACKET_START);
      packetAvailable = false;
      packetLength = 0;
    }

    if (collectPacket) {                    //collect packet data
      packet[packetLength++] = ch;
    }

    if (packetLength == PACKET_LENGTH) {    //finalise
      collectPacket = false;
      packetAvailable = true;
    }
  }

  if (packetAvailable) {
    swSer.print("I received: ");
    for (int i = 0; i < PACKET_LENGTH; i++) {
      swSer.print(packet[i]);
    }
  }
  packetAvailable = false; 
}
