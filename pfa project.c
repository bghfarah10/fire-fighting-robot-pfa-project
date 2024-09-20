
#include <Servo.h>

Servo myservo;

// Motor connections
const int LMS = 11;   // Left motor speed
const int RMS = 3;    // Right motor speed
const int LM1 = 9;    // Left motor direction 1
const int LM2 = 8;    // Left motor direction 2
const int RM1 = 7;    // Right motor direction 1
const int RM2 = 6;    // Right motor direction 2

// Flame sensor connections
const int leftFlameSensor = A0;
const int centerFlameSensor = A1;
const int rightFlameSensor = A2;

const int MOTOR_SPEED = 255; // Full speed

void setup() {
  pinMode(LMS, OUTPUT);
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RMS, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);

  myservo.attach(10);
  myservo.write(90);

  Serial.begin(9600); // For debugging
}

void move_motors(int leftSpeed, int rightSpeed) {
  // Set direction for left motor
  digitalWrite(LM1, LOW);
  digitalWrite(LM2, HIGH);
  // Set direction for right motor
  digitalWrite(RM1, LOW);
  digitalWrite(RM2, HIGH);
  // Set speed for both motors
  analogWrite(LMS, leftSpeed);
  analogWrite(RMS, rightSpeed);
}

void move_servo() {
  for (int pos = 0; pos <= 180; pos += 1) { // Move from 0 to 180 degrees
    myservo.write(pos);
    delay(15);
  }
  for (int pos = 180; pos >= 0; pos -= 1) { // Move from 180 to 0 degrees
    myservo.write(pos);
    delay(15);
  }
}

void loop() {
  int leftFlame = analogRead(leftFlameSensor);
  int centerFlame = analogRead(centerFlameSensor);
  int rightFlame = analogRead(rightFlameSensor);

  Serial.print("Left Flame: ");
  Serial.print(leftFlame);
  Serial.print(" Center Flame: ");
  Serial.print(centerFlame);
  Serial.print(" Right Flame: ");
  Serial.println(rightFlame);

  // Adjust motor speed based on flame sensor readings
  if (centerFlame < 500) {
    // Fire detected in front
    move_motors(MOTOR_SPEED, MOTOR_SPEED);
  } else if (leftFlame < 500) {
    // Fire detected on the left
    move_motors(0, MOTOR_SPEED); // Turn left
  } else if (rightFlame < 500) {
    // Fire detected on the right
    move_motors(MOTOR_SPEED, 0); // Turn right
  } else {
    // No fire detected
    move_motors(0, 0); // Stop
  }

  move_servo(); // Keep the servo moving
  delay(100); // Add a small delay for stability
}