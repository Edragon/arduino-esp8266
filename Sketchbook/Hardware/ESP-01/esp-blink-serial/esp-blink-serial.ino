
// build in LED
#define led 2


void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
}


void loop() {
  Serial.println("test");
  digitalWrite(led, HIGH);  
  delay(2000);

  digitalWrite(led, LOW);    
  delay(2000);                      

}
