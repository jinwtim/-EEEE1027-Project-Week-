// Pin definitions
const int trigPin = 3;    // Ultrasonic sensor Trigger pin
const int echoPin = 9;    // Ultrasonic sensor Echo pin

// Motor pins (modify according to your motor controller)
const int motor1Pin1 = 5; // IN1 on motor driver for motor 1
const int motor1Pin2 = 6; // IN2 on motor driver for motor 1
const int motor2Pin1 = 10; // IN3 on motor driver for motor 2
const int motor2Pin2 = 11; // IN4 on motor driver for motor 2

// LED pins
const int blueLedPin = 7;  // Blue LED
const int greenLedPin = 8; // Green LED

// Buzzer pin
const int buzzerPin = 4; // Buzzer

// Threshold distances in cm
const int distanceBlue = 30;
const int distanceGreen = 20;
const int distanceStop = 10;

// Slow motor speed
const int motorSpeed = 50; // Adjust this value for desired slow speed (0–255)

// Function to control motor speed and direction
void moveForward() {
  analogWrite(motor1Pin1, motorSpeed);
  analogWrite(motor1Pin2, 0);
  analogWrite(motor2Pin1, motorSpeed);
  analogWrite(motor2Pin2, 0);
}

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
}

// Function to calculate distance using ultrasonic sensor
long getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // Convert to cm
  return distance;
}

// Function to ring the buzzer
void ringBuzzer() {
  digitalWrite(buzzerPin, HIGH);
  delay(100); // Ring for 100 ms
  digitalWrite(buzzerPin, LOW);
}

void setup() {
  // Set pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  long distance = getDistance();

  // Debugging: print distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= distanceStop) {
    // Stop motors, turn off LEDs, and ring buzzer
    stopMotors();
    digitalWrite(blueLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    ringBuzzer();
    Serial.println("Obstacle too close! Stopping...");
  } else if (distance <= distanceGreen) {
    // Show green LED, ring buzzer, and move motors
    digitalWrite(blueLedPin, LOW);
    if (digitalRead(greenLedPin) == LOW) { // Ensure buzzer rings only once
      ringBuzzer();
    }
    digitalWrite(greenLedPin, HIGH);
    moveForward();
    Serial.println("Green light: Obstacle within 20 cm.");
  } else if (distance <= distanceBlue) {
    // Show blue LED, ring buzzer, and move motors
    digitalWrite(greenLedPin, LOW);
    if (digitalRead(blueLedPin) == LOW) { // Ensure buzzer rings only once
      ringBuzzer();
    }
    digitalWrite(blueLedPin, HIGH);
    moveForward();
    Serial.println("Blue light: Obstacle within 30 cm.");
  } else {
    // Clear path, no LEDs
    digitalWrite(blueLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
    moveForward();
    Serial.println("Clear path: Moving forward.");
  }

  delay(100); // Delay for stability
}