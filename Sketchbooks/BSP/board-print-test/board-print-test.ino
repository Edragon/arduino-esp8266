
void setup() {
  Serial.begin(9600);
  pinMode(2,OUTPUT);

}


void loop() {
  Serial.println("test1");
  digitalWrite(2, HIGH);
  delay(1000);       
  
  Serial.println("test2");
  digitalWrite(2, LOW);
  delay(1000);   
  
}
