//GridEye Tracking Code combined with Remote Control Code - ECSE 398 Project by Leo, Adam, and Jerry.
#include <Wire.h> // I2C Library
#include <SparkFun_GridEYE_Arduino_Library.h> // Grideye Library
#include <ESP32Servo.h> // Servo Library for ESP32
#include <Adafruit_AMG88xx.h> // Include the library for Panasonic Grid-EYE sensor

// Grid-EYE setup
GridEYE grideye; // Initializes sensor
float tempArray[64]; // Float array to store 64 pixels (since GridEye outputs 8x8 grid)

// Servo Parameters
Servo myservo;
int pos = 90;
const int servoPin = 2; // Pin for Servo PWM
int minServoAngle = 0;    // Minimum servo angle
int maxServoAngle = 180;  // Maximum servo angle

// PID Controller 
double Kp, Ki, Kd;
double dt, last_time;
double integral, previous, output = 0;
double setpoint = 4.5; // Desired center value

// Remote Control Parameters
const int powerButtonPin = 17;  // Power button
const int fanSpeedButtonPin = 16;  // Fan speed control button
const int directionUpButtonPin = 22;  // Up button
const int directionDownButtonPin = 21;  // Down button
const int directionLeftButtonPin = 19;  // Left button
const int directionRightButtonPin = 23;  // Right button
const int trackingToggleButtonPin = 25;  // Button to enable/disable tracking

const int motorPinLeft = 26;  // Left motor
const int motorPinRight = 27;  // Right motor
const int motorPinUp = 14;  // Up motor
const int motorPinDown = 13;  // Down motor
const int powerPin = 32;  // Pin to control system power
const int fanSpeedPin = 33;  // PWM pin for fan speed

// State variables
bool systemPower = false;      // System power state (on/off)
int fanSpeed = 128;            // Fan speed (0-255 for PWM)
bool trackingEnabled = false;  // Tracking (automatic mode) enabled/disabled

void setup() {
  Wire.begin();  // I2C
  grideye.begin();

  Serial.begin(115200); 

  // Initialize buttons for manual control
  pinMode(powerButtonPin, INPUT_PULLUP);
  pinMode(fanSpeedButtonPin, INPUT_PULLUP);
  pinMode(directionUpButtonPin, INPUT_PULLUP);
  pinMode(directionDownButtonPin, INPUT_PULLUP);
  pinMode(directionLeftButtonPin, INPUT_PULLUP);
  pinMode(directionRightButtonPin, INPUT_PULLUP);
  pinMode(trackingToggleButtonPin, INPUT_PULLUP);

  // Set motor pins as outputs
  pinMode(motorPinLeft, OUTPUT);
  pinMode(motorPinRight, OUTPUT);
  pinMode(motorPinUp, OUTPUT);
  pinMode(motorPinDown, OUTPUT);
  pinMode(powerPin, OUTPUT);
  pinMode(fanSpeedPin, OUTPUT);

  // Initialize PWM for fan speed control
  ledcAttach(fanSpeedPin, 5000, 8);  // Attach GPIO33 to PWM, 5kHz frequency, 8-bit resolution

  // Initialize the servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo.setPeriodHertz(50); // Standard 50 Hz servo
  myservo.attach(servoPin, 544, 2400); // Attach servo to pin 2

  // PID Variables
  Kp = 3;
  Ki = 0.75;
  Kd = 0.005;
  last_time = 0;

  // Start with system OFF
  digitalWrite(powerPin, LOW);
}

void loop() { 
  // Handle power button
  if (digitalRead(powerButtonPin) == LOW) {
    delay(300);  // Debounce
    systemPower = !systemPower;
    digitalWrite(powerPin, systemPower ? HIGH : LOW);
    Serial.println(systemPower ? "System ON" : "System OFF");
  }

  // Handle fan speed button
  if (digitalRead(fanSpeedButtonPin) == LOW) {
    delay(300);  // Debounce
    fanSpeed = (fanSpeed == 255) ? 128 : fanSpeed + 64;  // Cycle through fan speeds
    ledcWrite(fanSpeedPin, fanSpeed);  // Adjust the PWM output for fan speed
    Serial.println("Fan speed set to: " + String(fanSpeed));
  }

  // Handle manual directional controls using the servo
  if (systemPower) {
    if (digitalRead(directionUpButtonPin) == LOW) {
      pos = constrain(pos + 10, minServoAngle, maxServoAngle);  // Move up by increasing the position
      myservo.write(pos);
      delay(1000); // Adjust duration based on desired movement
      Serial.println("Moving Up");
    }

    if (digitalRead(directionDownButtonPin) == LOW) {
      pos = constrain(pos - 10, minServoAngle, maxServoAngle);  // Move down by decreasing the position
      myservo.write(pos);
      delay(1000);
      Serial.println("Moving Down");
    }

    if (digitalRead(directionLeftButtonPin) == LOW) {
      pos = minServoAngle;  // Move servo to minimum angle (simulate left)
      myservo.write(pos);
      delay(1000);
      Serial.println("Moving Left");
    }

    if (digitalRead(directionRightButtonPin) == LOW) {
      pos = maxServoAngle;  // Move servo to maximum angle (simulate right)
      myservo.write(pos);
      delay(1000);
      Serial.println("Moving Right");
    }
  }

  // Handle tracking toggle button
  if (digitalRead(trackingToggleButtonPin) == LOW) {
    delay(300);  // Debounce
    trackingEnabled = !trackingEnabled;
    Serial.println(trackingEnabled ? "Tracking Enabled" : "Tracking Disabled");
  }

  // Only run Grid-EYE tracking if tracking is enabled and system power is ON
  if (trackingEnabled && systemPower) {
    float totalTemp = 0; 
    float tempX = 0;
    float tempY = 0;

    for (int i = 0; i < 64; i++) {
      tempArray[i] = grideye.getPixelTemperature(i); // Read pixel value at current i

      int x = (i % 8) + 1;
      int y = (i / 8) + 1;

      float temp = tempArray[i] - 20;
      totalTemp += temp;
      tempX += x * temp;
      tempY += y * temp;
    }

    float centerX = tempX / totalTemp;
    float centerY = tempY / totalTemp;

    // PID Calculation and Servo Adjustment
    double current_time = millis();
    dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    double error = centerX - setpoint;
    output = pid(error);
    pos = pos + output;
    pos = constrain(pos, minServoAngle, maxServoAngle);
    myservo.write(pos);

    Serial.print("Output:");
    Serial.print(output);
    Serial.println();
  }

  delay(100);
}

// PID Error Function
double pid(double error) {
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  return (Kp * proportional) + (Ki * integral) + (Kd * derivative);
}
