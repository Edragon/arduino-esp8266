
void setup() {
	Serial.begin(4800);
	Serial.println("\nSoftware serial test started");
}

void loop() {
  
	while (Serial.available() > 0) {
		Serial.write(Serial.read());
	}

}
