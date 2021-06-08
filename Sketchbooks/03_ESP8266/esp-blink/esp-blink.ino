#define relay1 12
#define relay2 13
#define relay3 14
#define buzzer 16


void setup() {
  Serial.begin(115200);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
}


void loop() {
  Serial.println("test");
  digitalWrite(relay1, HIGH);  
  digitalWrite(relay2, HIGH);   
  digitalWrite(relay3, HIGH);  
  delay(5000);

  digitalWrite(relay1, LOW);   
  digitalWrite(relay2, LOW);   
  digitalWrite(relay3, LOW);   
  delay(5000);                      

}
