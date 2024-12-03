// ESP-12F: DIO, 4M, 3M SPIFF
// 

#define relay1 12
#define relay2 13
#define moduleLED 2
#define boardLED 16

void setup() {

  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(moduleLED, OUTPUT);
  pinMode(boardLED, OUTPUT);

}

void repeat() {
  digitalWrite(relay1, HIGH);  
  digitalWrite(relay2, HIGH);  
  digitalWrite(moduleLED, HIGH);  
  digitalWrite(boardLED, HIGH);  
  delay (1000);

  digitalWrite(relay1, LOW);  
  digitalWrite(relay2, LOW);  
  digitalWrite(moduleLED, LOW);  
  digitalWrite(boardLED, LOW);  
  delay (500);

}

// the loop function runs over and over again forever
void loop() {
  repeat();
}
