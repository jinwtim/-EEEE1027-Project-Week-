// Pin definitions for motor control
const int motor1Pin1 = 5; // IN1 on motor driver for motor 1
const int motor1Pin2 = 6; // IN2 on motor driver for motor 1
const int motor2Pin1 = 10; // IN3 on motor driver for motor 2
const int motor2Pin2 = 11; // IN4 on motor driver for motor 2

char command; // Variable to store the received command

void setup() {
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    Serial.begin(9600); // Start communication with the HC-05
}

void loop() {
    if (Serial.available()) {
        command = Serial.read(); // Read the command

        // Handle the command
        switch (command) {
            case 'F': moveForward();Serial.print(command); break;
            case 'B': moveBackward();Serial.print(command); break;
            case 'L': turnLeft();Serial.print(command); break;
            case 'R': turnRight();Serial.print(command); break;
            case 'S': stopMotors();Serial.print(command); break;
            default: break;
        }
    }
}

// Movement functions
void moveForward() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
}

void moveBackward() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

void turnLeft() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
}

void turnRight() {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
}

void stopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
}