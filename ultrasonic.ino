// Pin definitions
const int trigPin = 3;    // Ultrasonic sensor Trigger pin
const int echoPin = 9;   // Ultrasonic sensor Echo pin

// Motor pins (modify according to your motor controller)
const int motor1Pin1 = 5; // IN1 on motor driver for motor 1
const int motor1Pin2 = 6; // IN2 on motor driver for motor 1
const int motor2Pin1 = 10; // IN3 on motor driver for motor 2
const int motor2Pin2 = 11; // IN4 on motor driver for motor 2

// Threshold distance in cm
const int thresholdDistance = 30;

// Function to control motor direction
void moveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
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

void setup() {
  // Set pin modes
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Start serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  long distance = getDistance();

  // Debugging: print distance
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  if (distance <= thresholdDistance) {
    stopMotors(); // Stop the motors if the obstacle is close
    Serial.println("Obstacle detected! Stopping...");
  } else {
    moveForward(); // Keep moving if no obstacle
  }

  delay(100); // Delay for stability
}