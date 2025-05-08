
const int ENA = 5;  // PWM for speed for Motor 1
const int IN1 = 0;  // Direction for Motor 1 (IN2_Motor1 is inverted in hardware)

// Pins for Motor 2
const int ENB = 4; // PWM for speed for Motor 2
const int IN2 = 2; // Direction pin 1 for Motor 2

void setup() {
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN2, OUTPUT);
}

void loop() {
    // Both Motors Forward
    digitalWrite(IN1, HIGH);  // Motor 1 Forward (IN2_Motor1 becomes LOW due to inverter)
    analogWrite(ENA, 200);    // Set speed for Motor 1

    digitalWrite(IN2, HIGH); // Motor 2 Forward
    analogWrite(ENB, 200);    // Set speed for Motor 2
    delay(2000);

    // Both Motors Stop
    analogWrite(ENA, 0);      // Stop Motor 1
    analogWrite(ENB, 0);      // Stop Motor 2
    delay(1000);

    // Both Motors Backward
    digitalWrite(IN1, LOW);   // Motor 1 Backward (IN2_Motor1 becomes HIGH due to inverter)
    analogWrite(ENA, 200);    // Set speed for Motor 1

    digitalWrite(IN2, LOW);  // Motor 2 Backward
    analogWrite(ENB, 200);    // Set speed for Motor 2
    delay(2000);

    // Both Motors Stop
    analogWrite(ENA, 0);      // Stop Motor 1
    analogWrite(ENB, 0);      // Stop Motor 2
    delay(1000);
}
