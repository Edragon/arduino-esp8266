#include <Adafruit_NeoPixel.h>

// Define pins for each RC channel
int aileronPin = 14;   // Channel 1 (Throttle)
int elevatorPin = 12;  // Channel 2 (Steering)

const int ENA = 5; // PWM for speed for Motor 1
const int ENB = 4; // PWM for speed for Motor 2

const int IN1 = 0; // Direction for Motor 1
const int IN2 = 2; // Direction pin 1 for Motor 2

// WS2812 LED Strip Configuration
#define LED_PIN 15
#define LED_COUNT 8
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

long aileronControl;  // Mapped value from aileron channel (0-100)
long elevatorControl; // Mapped value from elevator channel (0-100)

// Reads the PWM signal from the aileron channel and maps it to 0-100
long readAileronControlSignal() {
    unsigned long rawPWM = pulseIn(aileronPin, HIGH, 25000);
    // If signal is lost (timeout) or clearly out of valid RC pulse range, return neutral (50)
    if (rawPWM == 0 || rawPWM < 900 || rawPWM > 2100) { 
        return 50; // Mid-point for 0-100 scale
    }
    long constrainedPWM = constrain(rawPWM, 1000, 2000);
    return map(constrainedPWM, 1000, 2000, 0, 100);
}

// Reads the PWM signal from the elevator channel and maps it to 0-100
long readElevatorControlSignal() {
    unsigned long rawPWM = pulseIn(elevatorPin, HIGH, 25000);
    // If signal is lost (timeout) or clearly out of valid RC pulse range, return neutral (50)
    if (rawPWM == 0 || rawPWM < 900 || rawPWM > 2100) {
        return 50; // Mid-point for 0-100 scale
    }
    long constrainedPWM = constrain(rawPWM, 1000, 2000);
    return map(constrainedPWM, 1000, 2000, 0, 100);
}

void setup() {
    pinMode(aileronPin, INPUT);
    pinMode(elevatorPin, INPUT); 

    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    // Initialize motors to off
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS (max = 255)
}

// Helper function to control a single motor
// pwmVal: -255 (full backward) to 255 (full forward)
void setMotorOutput(int dirPin, int speedPin, int pwmVal) {
    if (pwmVal > 0) { // Forward
        digitalWrite(dirPin, HIGH);
        analogWrite(speedPin, pwmVal);
    } else if (pwmVal < 0) { // Backward
        digitalWrite(dirPin, LOW);
        analogWrite(speedPin, -pwmVal); // Speed is positive
    } else { // Stop
        digitalWrite(dirPin, LOW); 
        analogWrite(speedPin, 0);
    }
}

// Function to set LED colors based on motor PWM
void setMotorLEDs(long motorPwm, int startLed, int numLeds) {
    uint32_t color = strip.Color(0, 0, 0); // Default to off
    int brightness = abs(motorPwm);

    if (motorPwm > 0) { // Forward
        color = strip.Color(0, brightness, 0); // Green
    } else if (motorPwm < 0) { // Backward
        color = strip.Color(brightness, 0, 0); // Red
    }

    for (int i = 0; i < numLeds; i++) {
        strip.setPixelColor(startLed + i, color);
    }
}

void loop() {
    // Read mapped control signals from each channel
    aileronControl = readAileronControlSignal();   // Throttle (0-100, 50 is neutral)
    elevatorControl = readElevatorControlSignal(); // Steering (0-100, 50 is neutral)

    // Map control values directly
    // aileronControl (0-100) to throttleValue (-255 to 255)
    long throttleValue = map(aileronControl, 0, 100, -255, 255);
    
    // elevatorControl (0-100) to steeringValue (-255 to 255)
    long steeringValue = map(elevatorControl, 0, 100, -255, 255);

    // Mix throttle and steering for differential drive
    long motor1Pwm = throttleValue + steeringValue;
    long motor2Pwm = throttleValue - steeringValue;

    // Constrain PWM values to the valid range [-255, 255]
    motor1Pwm = constrain(motor1Pwm, -255, 255);
    motor2Pwm = constrain(motor2Pwm, -255, 255);

    // Set motor speeds and directions
    setMotorOutput(IN1, ENA, motor1Pwm); // Motor 1
    setMotorOutput(IN2, ENB, motor2Pwm); // Motor 2

    // Update LEDs
    setMotorLEDs(motor1Pwm, 0, 4); // First 4 LEDs for Motor 1
    setMotorLEDs(motor2Pwm, 4, 4); // Last 4 LEDs for Motor 2
    strip.show();

    delay(20);  // Delay for responsiveness
}
