// GridEye Tracking Code - ECSE 398 Project by Leo, Adam, and Jerry

#include <Wire.h>                              // I2C Library
#include <SparkFun_GridEYE_Arduino_Library.h>  // GridEye Library
#include <ESP32Servo.h>                        // Servo Library for ESP32

// Grid-EYE Parameters
GridEYE grideye;      // Initializes sensor
float tempArray[64];  // Array to store 64 pixels (8x8 grid)
float minTemp = 20;   // Minimum temperature expected, updated by minFunc()

// Servo Parameters
Servo x_servo;  // Servo for X-axis movement
int xPos = 90;  // Initial position for X-axis servo
const int xServoPin = 0;  // GPIO0 for the servo signal pin
int minServoAngle = 5;    // Minimum servo angle
int maxServoAngle = 180;  // Maximum servo angle

// PID Controller Parameters
double Kp, Ki, Kd;
double dt, last_time;
double integral, previous, output = 0;
double setpoint = 4.5;  // Desired center position
double deadband = 0.1;  // Deadband to prevent micro-movements

// Buttons / Switches
const int directionPins[2] = {34, 35};  // Left and Right button pins
bool directionStates[2];  // Array to store button states
const int togglePin = 39; // Toggle switch for automatic/manual mode

void setup() {
  Wire.begin();           // I2C setup
  grideye.begin();        // Grid-EYE initialization

  Serial.begin(115200);   // Start serial for debugging

  // Servo Setup
  ESP32PWM::allocateTimer(0);  // Allocate timer for PWM on ESP32
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  x_servo.setPeriodHertz(50);  // Standard 50 Hz for servo
  x_servo.attach(xServoPin, 544, 2400);  // Attach servo to GPIO0 (min and max pulse width)

  // PID Variables
  Kp = 5;
  Ki = 0.50;
  Kd = 0.005;
  last_time = 0;

  // Button Setup
  for (int i = 0; i < 2; i++) {
    pinMode(directionPins[i], INPUT);  // Set each button pin as an input
  }
  pinMode(togglePin, INPUT);  // Toggle switch for auto/manual mode
}

void loop() {
  // Check the toggle switch state for automatic/manual mode
  bool isAutomaticMode = digitalRead(togglePin) == HIGH;

  if (isAutomaticMode) {
    TrackingFunc();  // Run automatic tracking
  } else {
    // Manual control
    for (int i = 0; i < 2; i++) {
      directionStates[i] = digitalRead(directionPins[i]);  // Read each button state
      if (directionStates[i] == LOW) {                     // If button is pressed
        switch (i) {
          case 0:  // Left button
            Serial.println("Left ON");
            xPos += 5;  // Move left
            xPos = constrain(xPos, minServoAngle, maxServoAngle);
            x_servo.write(xPos);
            break;
          case 1:  // Right button
            Serial.println("Right ON");
            xPos -= 5;  // Move right
            xPos = constrain(xPos, minServoAngle, maxServoAngle);
            x_servo.write(xPos);
            break;
        }
      }
    }
  }

  delay(100);  // Short delay for debounce and smoother control
}

// Grid-EYE Tracking Function (X-axis only)
void TrackingFunc() {
  minTemp = minFunc();  // Update minimum temperature
  Serial.print("Min temp:");
  Serial.println(minTemp);

  float totalTemp = 0;
  float tempX = 0;  // Initialize X temperature

  // Calculate Center of Temperature (CoT) in X-axis only
  for (int i = 0; i < 64; i++) {
    tempArray[i] = grideye.getPixelTemperature(i);  // Read each pixel temperature

    int x = (i % 8) + 1;  // Get X position (1 to 8)

    float temp = tempArray[i] - minTemp;  // Offset temperature by minTemp
    totalTemp += temp;  // Increment total temperature
    tempX += x * temp;  // Weighted X position for CoT
  }

  float centerX = tempX / totalTemp;  // Calculate X center

  // Calculate error and adjust servo position
  unsigned long current_time = millis();
  dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  double error = centerX - setpoint;
  output = pid(error);
  xPos += output;
  xPos = constrain(xPos, minServoAngle, maxServoAngle);  // Constrain to min/max servo angles
  x_servo.write(xPos);  // Write the new position to the servo

  Serial.print("Output:");
  Serial.println(output);
}

// PID Error Function
double pid(double error) {
  double proportional = error;

  if (abs(error) > deadband) {
    integral += error * dt;
    integral = constrain(integral, -20, 20);
  } else {
    integral = 0;
  }

  double derivative = (error - previous) / dt;
  previous = error;
  derivative = constrain(derivative, -20, 20);

  double output = (Kp * proportional) + (Ki * integral) + (Kd * derivative);
  return output;
}

// Minimum Temperature Detection Function
float minFunc() {
  float minTemp = 20;
  for (int i = 0; i < 64; i++) {
    tempArray[i] = grideye.getPixelTemperature(i);
    if (tempArray[i] < minTemp) {
      minTemp = tempArray[i];
    }
  }
  return constrain(minTemp, 15, 20);
}