#define LED 13 
#define relay 12
void setup() {
  // put your setup code here, to run once:
  pinMode(LED, OUTPUT);
  pinMode(relay, OUTPUT);
}

void loop() {
  digitalWrite(LED, HIGH);   
  digitalWrite(relay, HIGH);   
  delay(1000);    
                     
  digitalWrite(LED, LOW);   
  digitalWrite(relay, LOW);   
  delay(1000);                      
  
}
