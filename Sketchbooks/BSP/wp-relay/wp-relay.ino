#define relay1 12
#define relay2 13
#define relay3 14


void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  
  Serial.begin(115200);
  Serial.println("115200");
}

// the loop function runs over and over again forever
void loop() {
  
  digitalWrite(relay1, HIGH);   
  digitalWrite(relay2, HIGH);   
  digitalWrite(relay3, HIGH);  
  Serial.println("115200");
  delay(1000);                  
     
  digitalWrite(relay1, LOW);   
  digitalWrite(relay2, LOW); 
  digitalWrite(relay3, LOW); 
  delay(1000);                     
}
