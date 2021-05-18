void setup() {

  //uint8_t C[] = "AT";       // Won't compile
  //int8_t C[] = "AT";        // Won't compile
  //unsigned char C[] = "AT"; // Won't compile
  //signed char C[] = "AT";   // Won't compile

  // Because I know someone will ask, the error is: "call of overloaded 'print(unsigned char [3])' is ambiguous"

  char C[] = "AT";            // This is the only version which will compile
 
  Serial.begin(115200);
  Serial.println(C);
  
  Serial.write(C,2);          // Both .write and .print behave the same

}

void loop() {

  
}
