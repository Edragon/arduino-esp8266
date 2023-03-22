

#include <SoftwareSerial.h>

#define BAUD_RATE 4800

int hlw[23];

// for wemos
// D5 is RX pin 14, D6 is TX pin
// D8 = IO15
// D7 = IO13
// D6 = IO12 TX

// D5 = IO14 RX


// HLW8032 TXD data pin should connect to D5

// Demo data should look like this

// F2 5A 02 DA 78 07 1A E0 00 3D 3B 03 EC F5 4C C4 58 9C 2A 39 61 00 00 79
// F2 5A 02 DA 78 07 1A E0 00 3D 3B 03 EC F5 4C C4 58 9D C2 9F 61 00 00 78

// please read more details on wiki page

SoftwareSerial swSer;


void setup() {
  Serial.begin(BAUD_RATE);
  swSer.begin(BAUD_RATE, SWSERIAL_8N1, 15, 12, false, 256);
}

bool read_data () {
  while (swSer.available() > 0) {
    int byte0 = swSer.read();
    Serial.write(byte0);
    yield();
  }
}

void loop() {
  read_data ();

}
