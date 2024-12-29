#include "CytronMotorDriver.h"
#include <Wire.h>
#include <Adafruit_LiquidCrystal.h>
#include <PinChangeInterrupt.h> 

// Initialize the LCD display (pins connected to the LCD)
Adafruit_LiquidCrystal lcd(8, 9, 7, 2, 3, 4);

// MPU6050 I2C address
const int MPU_addr = 0x68;

// Variables to store raw MPU data
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

// Timer for tracking elapsed time
unsigned long timeTaken;

// Calibration values for the accelerometer
int minVal = 265;
int maxVal = 402;

// Variables for calculating tilt angles and distance
float distance;
int distance_temp;
double x, y, z;

// Flags to control the robot's states
bool flag1 = true;
bool flag2 = false;
bool flag3 = false;
bool flag4 = false;
bool flag5 = false;

// IR sensor readings
int IR_left;
int IR_right;

// Encoder counts
volatile int leftEncoderCount = 0;   
volatile int rightEncoderCount = 0; 

// Encoder pins
const int encoderL = 13;  
const int encoderR = 12;  

// Wheel specifications
const float wheelCircumference = 20.106; // Circumference in cm
const int pulsesPerRevolution = 20;      // Encoder pulses per revolution

// Initialize Cytron motor drivers
CytronMD motorLeft(PWM_PWM, 10, 11);
CytronMD motorRight(PWM_PWM, 5, 6);

// Variables for gyroscope-based rotation
float angularZ = 0; 
unsigned long prevTime = 0;

// Function to read accelerometer data and calculate tilt angle
void MPU() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();

  // Map raw accelerometer data to angles
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  // Calculate the tilt angle in degrees
  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
}

// Function to read gyroscope data
void readGyroData() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);

  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

// Interrupt handler for the left encoder
void leftEncoderInterrupt() {
  leftEncoderCount++;
}

// Interrupt handler for the right encoder
void rightEncoderInterrupt() {
  rightEncoderCount++;
}

// Function to move the robot forward
void moveForward() {
  motorLeft.setSpeed(75);
  motorRight.setSpeed(85);
}

// Function to turn the robot right
void turnRight() {
  motorLeft.setSpeed(-180);
  motorRight.setSpeed(200);
}

// Function to turn the robot left
void turnLeft() {
  motorLeft.setSpeed(220);
  motorRight.setSpeed(-200);
}

// Function to stop the motors
void motorStop() {
  motorLeft.setSpeed(0);
  motorRight.setSpeed(0);
}

// Function to rotate the robot 360 degrees using the gyroscope
void rotate360() {
  angularZ = 0; 
  prevTime = millis();

  motorLeft.setSpeed(-255); 
  motorRight.setSpeed(255);

  // Loop until the robot completes a 360-degree rotation
  while (angularZ < 355) {
    readGyroData();

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0; 
    prevTime = currentTime;

    // Calculate angular rotation using gyroscope data
    float gyroZrate = GyZ / 131.0;

    angularZ += gyroZrate * deltaTime;

    Serial.print("Angle: ");
    Serial.println(angularZ);
  }

  motorStop(); // Stop the motors after rotation
}

void setup() {
  // Initialize the LCD
  lcd.begin(16, 2);
  lcd.setBacklight(1);

  // Set encoder pins as input
  pinMode(encoderL, INPUT_PULLUP);  
  pinMode(encoderR, INPUT_PULLUP);  

  // Attach interrupts for encoders
  attachPCINT(digitalPinToPCINT(encoderL), leftEncoderInterrupt, CHANGE);           
  attachInterrupt(digitalPinToInterrupt(encoderR), rightEncoderInterrupt, CHANGE); 

  // Initialize MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // Wake up MPU6050
  Wire.write(0);
  Wire.endTransmission(true);

  // Begin serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // State 1: Align the robot to a specific angle using accelerometer data
  while (flag1) {
    MPU();
    Serial.print("x: ");
    Serial.println(x);  
    if (x < 24 || x > 30) {
      motorLeft.setSpeed(80);
      motorRight.setSpeed(80);
    } 
    else if (x > 24 && x < 30) {
      int x_temp = x; 
      motorStop();
      lcd.print("Angle: ");
      lcd.print(x_temp, 1);
      delay(4000);
      flag2 = true;
      flag1 = false;
    }
  }
  
  // State 2: Move forward and rotate 360 degrees
  while (flag2) {
    MPU();
    Serial.print("x2: ");
    Serial.println(x);
    if (x > 2) {
      motorLeft.setSpeed(200);
      motorRight.setSpeed(200);
    } 
    else {
      motorStop();
      motorLeft.setSpeed(100);
      motorRight.setSpeed(100);
      delay(300);
      motorStop();
      delay(4000);
      rotate360(); 
      MPU();
      flag3 = true;
      flag2 = false;
    }
  }

  // State 3: Align to a specific angle range
  while (flag3) {
    MPU();
    Serial.print("x3: ");
    Serial.println(x);
    if (x < 240 || x > 260) {
      motorLeft.setSpeed(100);
      motorRight.setSpeed(80);
    } 
    else if (x > 240 && x < 260) {
      motorStop();
      flag4 = true;
      flag3 = false;
    }
  }
      
  // State 4: Final angle alignment
  while (flag4) {
    MPU();
    Serial.print("x4: ");
    Serial.println(x);
    if (x > 3) {
      motorLeft.setSpeed(100);
      motorRight.setSpeed(80);
    }
    else if (x < 3) {
      motorStop();
      delay(2000);      
      flag5 = true;
      flag4 = false;
    } 
  }  

  // State 5: Obstacle avoidance and distance tracking
  leftEncoderCount = -20;
  rightEncoderCount = -20;
  timeTaken = 0;

  while (flag5) {
    timeTaken = millis();
    timeTaken = timeTaken / 1000;
    IR_left = digitalRead(A2);
    IR_right = digitalRead(A3);

    if (IR_left == 1 && IR_right == 1) {
      moveForward();
    }
    else if (IR_left == 1 && IR_right == 0) {
      turnLeft();
    }
    else if (IR_left == 0 && IR_right == 1) {
      turnRight();
    }
    else if (IR_left == 0 && IR_right == 0) {
      motorStop();
      flag5 = false;
    }

    // Calculate distance traveled using encoder data
    float leftDistance = (leftEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;
    float rightDistance = (rightEncoderCount / (float)pulsesPerRevolution) * wheelCircumference;
    float distance = (leftDistance + rightDistance) / 2.0;
    distance_temp = int(distance);

    // Stop if a specific distance is reached
    if (distance_temp == 180) {
      motorStop();
      delay(2000);
    }

    // Display distance and time on the LCD
    lcd.setCursor(0, 0);
    lcd.print("Distance:");
    lcd.print(distance);
    lcd.print(" cm");

    lcd.setCursor(0, 1);
    lcd.print("Time: ");
    lcd.print(timeTaken);
    lcd.print("s");
  }
}
