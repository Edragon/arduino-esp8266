#define relay1 12
#define relay2 13
#define led1 2
#define led2 16

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  Serial.begin(115200);
  Serial.println("115200");
}

// the loop function runs over and over again forever
void loop() {

  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  digitalWrite(led1, HIGH);
  digitalWrite(led2, HIGH);
  Serial.println("115200");
  delay(2000);

  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  delay(2000);
}
