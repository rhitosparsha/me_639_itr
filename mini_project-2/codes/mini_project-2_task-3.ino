#include <Arduino.h>
#include <math.h>

const int M1_PwM = 14; // PWM pin for motor 1
const int M2_PwM = 13; // PWM pin for motor 2
const int M1_D1 = 27;  // Direction pin 1 for motor 1
const int M2_D1 = 12;  // Direction pin 1 for motor 2

const int encoder1PinA = 26; // Encoder 1 signal A pin
const int encoder1PinB = 25; // Encoder 1 signal B pin
const int encoder2PinA = 33; // Encoder 2 signal A pin
const int encoder2PinB = 32; // Encoder 2 signal B pin

const int pulsesPerRevolution = 8000; // PPR for the AMT1120

const float link1Length = 105.0; // Length of link 1 in mm
const float link2Length = 105.0; // Length of link 2 in mm

volatile int encoderPosition1 = 0;
volatile int encoderPosition2 = 0;

// Desired end effector positions (x, y) for the straight-line path
const float x_desired[] = {105, 0, -105}; 
const float y_desired[] = {10, 10, 10};   
float kp = 0.63; // Proportional gain
float kd = 0.04; // Derivative gain

float previousErrorX = 0.0; // Initialize previous error for X
float previousErrorY = 0.0; // Initialize previous error for Y

int currentPositionIndex = 0; // Index for the current desired position

void setup() {
  Serial.begin(9600);

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  // Attach interrupt handlers for encoder pulses
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1Interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2Interrupt, CHANGE);

  // Motor control pins setup
  pinMode(M1_D1, OUTPUT);
  pinMode(M1_D2, OUTPUT);
  pinMode(M2_D1, OUTPUT);
  pinMode(M2_D2, OUTPUT);
  pinMode(M1_PwM, OUTPUT);
  pinMode(M2_PwM, OUTPUT);
}

void loop() {
  // Calculate the current theta values
  float theta1 = radians(static_cast<float>(encoderPosition1) / pulsesPerRevolution * 360.0);
  float theta2 = radians(static_cast<float>(encoderPosition2) / pulsesPerRevolution * 360.0);
  
  // Calculate required joint angles using inverse kinematics
  float r = sqrt(x_desired * x_desired + y_desired * y_desired);
  float phi = atan2(y_desired, x_desired);

  float cos_theta2 = (r * r - link1Length * link1Length - link2Length * link2Length) / (2.0 * link1Length * link2Length);
  float sin_theta2 = sqrt(1.0 - cos_theta2 * cos_theta2);

  float Theta2 = atan2(sin_theta2, cos_theta2);
  float Theta1 = phi - atan2(link2Length * sin(theta2), link1Length + link2Length * cos(theta2));

  // Convert angles to degrees
  float theta1_req = degrees(Theta1);
  float theta2_req = degrees(Theta2);

  // Calculate the error between the current and desired positions
  float error_theta1 = theta1_req - theta1;
  float error_theta2 = theta2_req - theta2;
  
  
  // Calculate control inputs (PD control for position control)
  float control1 = kp1 * error_theta1 + kd * (error_theta1 - previousError1);
  float control2 = kp2 * error_theta2 + kd * (error_theta2 - previousError2);

  previousError1 = error_theta1;
  previousError2 = error_theta2;

  // Map control values to motor speeds (for position control)
  int motorSpeed1;
  int motorSpeed2;
  if (control1 >= 0) {
    motorSpeed1 = map(control1, 0, 100, 45, 79);
  } else {
    motorSpeed1 = map(control1, -100, 0, -79, -45);
  }
  if (control2 >= 0) {
    motorSpeed2 = map(control2, 0, 100, 40, 79);
  } else {
    motorSpeed2 = map(control2, -100, 0, -79, -40);
  }

  digitalWrite(M1_D1, motorSpeed1 > 0 ? HIGH : LOW); // Direction for motor 1 (upper arm)
  digitalWrite(M2_D1, motorSpeed2 > 0 ? HIGH : LOW); // Direction for motor 2 (lower arm)

  analogWrite(M1_PwM, abs(motorSpeed1)); // Set motor 1 speed
  analogWrite(M2_PwM, abs(motorSpeed2)); // Set motor 2 speed


  // Check if the end effector is close to the desired position
  if (abs(error_x) < 1.0 && abs(error_y) < 1.0) {
    // Move to the next desired position
    currentPositionIndex++;
    if (currentPositionIndex >= sizeof(x_desired) / sizeof(x_desired[0])) {
      currentPositionIndex = 0; // Loop back to the beginning
    }
  }

  // Print the current end effector position
  Serial.print("Current End Effector Position (x, y): ");
  Serial.print(x_current);
  Serial.print(", ");
  Serial.println(y_current);

  // Delay
  delay(500);
}

void handleEncoder1Interrupt() {
  // Read the current state of encoder1PinA
  int stateA = digitalRead(encoder1PinA);

  // Read the current state of encoder1PinB
  int stateB = digitalRead(encoder1PinB);

  // Determining the direction of rotation based on the states of A and B
  int direction = (stateA == stateB) ? 1 : -1;

  // Update encoder 1 position
  encoderPosition1 += direction;
}

void handleEncoder2Interrupt() {
  // Read the current state of encoder2PinA
  int stateA = digitalRead(encoder2PinA);

  // Read the current state of encoder2PinB
  int stateB = digitalRead(encoder2PinB);

  // Determining the direction of rotation based on the states of A and B
  int direction = (stateA == stateB) ? 1 : -1;

  // Update encoder 2 position
  encoderPosition2 += direction;
}
