//GridEye Tracking Code - ECSE 398 Project by Leo, Adam, and Jerry

#include <Wire.h>                              //I2C Library
#include <SparkFun_GridEYE_Arduino_Library.h>  //Grideye Library
#include <ESP32Servo.h>                        //Servo Library for ESP32


//GridEYE Parameters
GridEYE grideye;      //Initializes sensor
float tempArray[64];  //Float array to store 64 pixels (since GridEye outputs 8x8 grid). Switch to int type if you want to remove decimal points.
float minTemp = 20;   //Setting average room temperature as minTemp, for use in setup later


//Servo Parameters
Servo track_servo, x_servo, y_servo;
int pos = 90;
int xPos = 90;
int yPos = 90;
const int servoPin = 2;  //Pin for Servo PWM
const int xServoPin = 100;
const int yServoPin = 100;
int minServoAngle = 5;  // Minimum servo angle
int maxServoAngle = 180;  // Maximum servo angle


//PID Controller
double Kp, Ki, Kd;
double dt, last_time;
double integral, previous, output = 0;
double setpoint = 4.5;  //Desired center value
double deadband = 0.1;  //Deadband to prevent micro-movements


//Buttons / Switches
const int directionPins[4] = { 34, 33, 35, 39 };
bool directionStates[4];  //Array to store button states
const int togglePin = 100;


void setup() {

  Wire.begin();  // I2C
  grideye.begin();

  Serial.begin(115200);

  //Servo Setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);               //Allow allocation of all timers
  track_servo.setPeriodHertz(50);           // standard 50 hz servo
  track_servo.attach(servoPin, 544, 2400);  //default values almost (0-180), little more added on 0 side to prevent current draw

  x_servo.setPeriodHertz(50);
  x_servo.attach(xServoPin, 544, 2400);

  //y_servo.setPeriodHertz(50);
  //y_servo.attach(yServoPin, 544, 2400);


  //PID Variables
  Kp = 5;
  Ki = 0.50;
  Kd = 0.005;
  last_time = 0;


  //Servo Button setup
  for (int i = 0; i < 4; i++) {
    pinMode(directionPins[i], INPUT);  // Set each button pin as an input
  }


  //Mintemp setup
  for (int i = 0; i < 64; i++) {  //Loop that goes through each pixel and assigns it to the temperature array
    tempArray[i] = grideye.getPixelTemperature(i);
    if (tempArray[i] < minTemp) {
      minTemp = tempArray[i];  // Update minTemp with the new lowest temperature
    }
  }
  minTemp = constrain(minTemp, 15, 25);  //Constraints set up incase of glitches / unwanted temperature readings

}


void loop() {

//read toggle switch -> if in automatic, go to temperature reading lines
  bool isAutomaticMode = digitalRead(togglePin) == HIGH;

  if (isAutomaticMode) {
    gridEyeFunction();
  } else {
    for (int i = 0; i < 4; i++) {
    directionStates[i] = digitalRead(directionPins[i]);  // Read each button state
    if (directionStates[i] == LOW) {                     // If button is LOW
      directionFunction(i);
    }
    }
  }

  delay(100);
}


//GridEYE Measurements Function
void gridEyeFunction() {
  float totalTemp = 0;
  float tempX = 0;
  float tempY = 0;  //Resets temperature variables, placed before loop to initialize before new data gathered

  for (int i = 0; i < 64; i++) {                    //Loop that goes through each pixel and assigns it to the temperature array
    tempArray[i] = grideye.getPixelTemperature(i);  //Read pixel value at current i, store it in temperature array

    //tempPrintFunction(i); //To print GridEYE's temperature data, comment in/out this calling function

    int x = (i % 8) + 1;
    int y = (i / 8) + 1;  //+1 included so that the grid does not include 0 values, done for CoT calculation

    float temp = tempArray[i] - minTemp;  //Lowers value of temp array so that any slight change is more noticeable

    totalTemp += temp;  //Incrementing total temperature counter
    tempX += x * temp;  //Multiplies current temperature by its grid position for CoT calculation later on
    tempY += y * temp;
  }

  float centerX = tempX / totalTemp;
  float centerY = tempY / totalTemp;  //Gets x and y center temperature


  //Change in T calculation (for integral)
  unsigned long current_time = millis();             //Returns the number of milliseconds passed since the Arduino board began running the current program
  dt = (current_time - last_time) / 1000.00;  //Calculates change in time in seconds
  last_time = current_time;                   //Sets previous time for next iteration


  //Error + Servo Writing
  double error = centerX - setpoint;                   //Calculates error (distance between current centerX temp and setpoint, which is usually 4.5 (middle))
  output = pid(error);                                 //Goes to PID function to calculate error output
  pos += output;                                       //Calculates position for servo using 90 (middle) + error output
  pos = constrain(pos, minServoAngle, maxServoAngle);  //Constrains position variable between 0 and 180
  track_servo.write(pos);                              //Writes position to servo

  //debugFunction(centerX, error, output, pos, dt, current_time);
}



//PID Error Function
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

  //pidPrintFunction(double proportional, double integral, double derivative)

  return output;
}


//Direction Button Function
void directionFunction(int directionIndex) {
  switch (directionIndex) {
    case 0:
      //
      break;
    case 1:
      //
      break;
    case 2:
      Serial.print("Left ON");
      xPos += 5;
      xPos = constrain(xPos, minServoAngle, maxServoAngle);  
      x_servo.write(xPos); 
      break;
    case 3:
      Serial.print("Right ON");
      xPos -= 5;
      xPos = constrain(xPos, minServoAngle, maxServoAngle);  
      x_servo.write(xPos);      
      break;
  }
}



//Print GridEYE Data Function
void tempPrintFunction(int i) {
  Serial.print(tempArray[i]);  //Prints data from temperature array
  Serial.print(" ");           //Prints blank space between values to make them easier to read

  if ((i + 1) % 8 == 0) {  //If the ith iteration + 1 is divisible by 8, run code in the loop
    Serial.println();      //Start a new line (Since i goes up to 64, makes 8x8 grid of data in monitor)
  }

  if ((i + 1) ==  64) {
    Serial.println();  //Prints empty lines in serial monitor to make each dataset visually distinct
    Serial.println();
  }
}


//PID Error Debug Function
void pidPrintFunction(double proportional, double integral, double derivative) {
  Serial.println("------ PID Components ------");
  Serial.print("Proportional: ");
  Serial.println(proportional);
  Serial.print("Integral: ");
  Serial.println(integral);
  Serial.print("Derivative: ");
  Serial.println(derivative);
  Serial.println("---------------------------\n");
}


//General Debug Info Function
void debugFunction(float centerX, double error, double output, int pos, long dt, unsigned long current_time) {
  Serial.println("------ Debug Info ------");
  Serial.print("Center X: ");
  Serial.println(centerX);
  Serial.print("Error: ");
  Serial.println(error);
  Serial.print("Output: ");
  Serial.println(output);
  Serial.print("Pos: ");
  Serial.println(pos);
  Serial.print("DT: ");
  Serial.println(dt);
  Serial.println("------------------------\n");
}
