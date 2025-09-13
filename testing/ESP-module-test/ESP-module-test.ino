
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(2, OUTPUT);
  pinMode(16, OUTPUT);
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(2, HIGH);  // turn the LED on (HIGH is the voltage level)
  digitalWrite(16, HIGH);  // turn the LED on (HIGH is the voltage level)
  Serial.print("test started .. ");
  delay(1000);                     // wait for a second
  digitalWrite(2, LOW);  // turn the LED off by making the voltage LOW
  digitalWrite(16, LOW);  // turn the LED off by making the voltage LOW
  Serial.print("test started .. ");
  delay(1000);
  // wait for a second
}
