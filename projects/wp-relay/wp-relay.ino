#define relay1 12
#define relay2 13
#define relay3 14
#define LED1 15
#define LED2 2
#define buzzer 16

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(buzzer, OUTPUT);

  Serial.begin(115200);
  Serial.println("115200");
}

// the loop function runs over and over again forever
void loop() {

  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(relay3, HIGH);
  digitalWrite(LED1, HIGH);
  digitalWrite(LED2, HIGH);
  digitalWrite(buzzer, LOW);
  Serial.println("115200");
  delay(5000);

  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(relay3, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(buzzer, HIGH);
  delay(1000);
}
