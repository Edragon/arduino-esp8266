#define LED2 2


void setup() {
  // initialize digital pin LED_BUILTIN as an output.

  pinMode(LED2, OUTPUT);

  Serial.begin(115200);
  Serial.println("115200");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED2, HIGH);
  Serial.println("115200");
  delay(1000);

  digitalWrite(LED2, LOW);
  delay(1000);
}
