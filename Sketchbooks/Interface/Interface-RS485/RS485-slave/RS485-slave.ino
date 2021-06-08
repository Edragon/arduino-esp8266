int enablePin = 2; // ebanble pin for slave

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup() {
  // initialize serial:
  Serial.begin(9600);
  // setup enable pin for slave (always low)
  pinMode(enablePin, OUTPUT);
  delay(10);
  digitalWrite(enablePin, LOW);
  // set digital pin 8 - 11 for output
  for (int i = 8; i < 12; i++) {
    pinMode(i, OUTPUT);
  }
}

void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    // make sure this message for slave "A"
    if (inputString.charAt(0) == 'A') {  
      for (int i = 1; i < 5 ; i++) {
        if (inputString.charAt(i) == '1')
          digitalWrite(7 + i, HIGH);
        else
          digitalWrite(7 + i, LOW);
      }
    } 
    // clear the string:
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read(); 
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    } 
  }
}

