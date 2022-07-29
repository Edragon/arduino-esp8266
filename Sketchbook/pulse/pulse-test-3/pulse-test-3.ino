const byte interruptPin = 14;
volatile byte interruptCounter = 0;
int numberOfInterrupts = 0;

void ICACHE_RAM_ATTR handleInterrupt();

void setup() {

  Serial.begin(115200);
  //pinMode(interruptPin, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, FALLING);
  
  pinMode(interruptPin, LOW);
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);


}

void handleInterrupt() {
  interruptCounter++;
}

void loop() {

  if(interruptCounter>0){

      interruptCounter--;
      numberOfInterrupts++;

      Serial.print("An interrupt has occurred. Total: ");
      Serial.println(numberOfInterrupts);
  }

}
