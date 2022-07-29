

#define IR_S 14 // ir transmitter 
#define IR_R 5
#define BTN 13
#define LDE 4

void setup() {
  pinMode (IR_S, OUTPUT);
  pinMode (IR_R, INPUT);
  pinMode (LDE, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  Serial.println("115200");
  digitalWrite(IR_S, LOW);
  digitalWrite(LDE, LOW);
  delay(1000);

  digitalWrite(IR_S, HIGH);
  digitalWrite(LDE, HIGH);
  delay(1000);
}
