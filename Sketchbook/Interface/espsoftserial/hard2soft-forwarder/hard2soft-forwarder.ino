// IO 4 + 5

#include <SoftwareSerial.h>
SoftwareSerial soft_Serial;


void setup() {
    Serial.begin(4800); // HLW8032
    
    soft_Serial.begin(4800, SWSERIAL_8N1, 4, 5, false, 95, 11);  // Output debug

    Serial.println( PSTR("\nSoftware serial test started") );

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

// output total 24 bytes
// F2 5A 02 E0 B8  07 34 2D 00 3E  67 07 22 DC 4D  ED 38 DF EC 33  61 00 00 7D 
// F2 5A 02 E0 B8 07 34 2D 00 3E 67 07 22 DC 4D ED 38 DF EC 33 61 00 00 7D 

// 55 5A 02 E0 B8 07 42 D0 00 3E 67 04 71 D2 4D ED 38 67 1C 7C 61 00 00 71
