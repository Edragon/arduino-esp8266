int enablePin = 2;  // use pin 2 for enable pin 

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // setup enable pin for server (always high)
  pinMode(enablePin, OUTPUT);
  delay(10);
  digitalWrite(enablePin, HIGH);
}

void loop() {
  // send message to slave ("A") with different value
  for (int i = 0; i < 5; i++) {
    switch (i) {
      case 0 : Serial.println("A1000"); break;
      case 1 : Serial.println("A0100"); break;
      case 2 : Serial.println("A0010"); break;
      case 3 : Serial.println("A0001"); break;
    }
    delay(2000);
  }
}
