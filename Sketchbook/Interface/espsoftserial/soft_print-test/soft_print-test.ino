
#include <SoftwareSerial.h>


SoftwareSerial soft_Serial;


void setup() {
    Serial.begin(115200);
    Serial.println(PSTR("\nSoftware serial test started"));

    
    soft_Serial.begin(57600, SWSERIAL_8N1, 4, 5, false, 95, 11);

}

void loop() {
    for (char ch = ' '; ch <= 'z'; ch++) {
        soft_Serial.write(ch);
    }
    soft_Serial.println();
    delay(1000);
}
