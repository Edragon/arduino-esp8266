// IO 4 and 5

#include <SoftwareSerial.h>

#define D5 (14)
#define D6 (12)
#define D7 (13)
#define D8 (15)
#define TX (1)
#define BAUD_RATE 57600


// Reminder: the buffer size optimizations here, in particular the isrBufSize that only accommodates
// a single 8N1 word, are on the basis that any char written to the loopback SoftwareSerial adapter gets read
// before another write is performed. Block writes with a size greater than 1 would usually fail. 

// 14 rx 12 tx 
SoftwareSerial swSer;
//SoftwareSerial swSer(14, 12, false, 95, 10);

void setup() {
  Serial.begin(4800);
  
  swSer.begin(BAUD_RATE, SWSERIAL_8N1, D5, D6, false, 95, 11);


  // ESP8266 internal cache RAM needs warm up - allow write and ISR to load
  swSer.write(static_cast<uint8_t>(0));
  Serial.println("\nSoftware serial test started");

  for (char ch = ' '; ch <= 'z'; ch++) {
    swSer.write(ch);
  }
  swSer.println("");
}

void loop() {
  while (swSer.available() > 0) {
    Serial.write(swSer.read());
    yield();
  }
  while (Serial.available() > 0) {
    swSer.write(Serial.read());
    yield();
  }

}
