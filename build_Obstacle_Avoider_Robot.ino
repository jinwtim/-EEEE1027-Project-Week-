#include <Servo.h>

// Pin definitions
const int trigPin = 4;
const int echoPin = 13;
const int motor1Pin1 = 5; // Motor 1 input pins
const int motor1Pin2 = 6;
const int motor2Pin1 = 10; // Motor 2 input pins
const int motor2Pin2 = 11;
const int servoPin = 12;

// Variables
Servo servo;
long duration;
int distance;
int safeDistance = 20; // Safe distance in cm

void setup() {
  // Motor pins as output
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  
  // Ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // Initialize servo
  servo.attach(servoPin);
  servo.write(90); // Center position
  
  Serial.begin(9600);
}

void loop() {

  
  distance = getDistance();
  
  if (distance < safeDistance) {
    stopMotors();
    reverseMotors();
    delay(1000);
    stopMotors();
    findSafePath();
  } else {
    moveForward();
  }
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Convert to cm
}

void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
}

void reverseMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
}

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

void findSafePath() {
  int leftDistance, rightDistance;
  
  // Scan left
  servo.write(0);
  delay(500);
  leftDistance = getDistance();
  
  // Scan right
  servo.write(180);
  delay(500);
  rightDistance = getDistance();
  
  // Return to center
  servo.write(90);
  delay(500);
  
  if (leftDistance > safeDistance && leftDistance >= rightDistance) {
    turnLeft();
  } else if (rightDistance > safeDistance) {
    turnRight();
  } else {
    reverseMotors();
    delay(1000);
  }
}

void turnLeft() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  delay(500); // Adjust turning time
  stopMotors();
}

void turnRight() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  delay(500); // Adjust turning time
  stopMotors();
}