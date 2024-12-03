
// FS none ~1024 OTA
// 921600
// DOUT
// button A should be pre-set to high level 1

int pushButton = 0;

// 
void setup() {
  // 
  Serial.begin(115200);
  // 
  pinMode(pushButton, INPUT);
}

// 
void loop() {
  // 
  int buttonState = digitalRead(pushButton);
  // 
  Serial.println(buttonState);
  delay(1);        // 
}
