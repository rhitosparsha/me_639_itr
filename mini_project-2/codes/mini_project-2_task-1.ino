#include <Arduino.h>
#include <math.h>

const int M1_PwM = 14; // PWM pin for motor 1
const int M2_PwM = 13; // PWM pin for motor 2
const int M1_D1 = 27; // Direction pin 1 for motor 1
const int M1_C = 35;  // current sensor pin for motor 1
const int M2_D1 = 12; // Direction pin 1 for motor 2
const int M2_C = 34;  // current sensor pin for motor 2

const int encoder1PinA = 26; // Encoder 1 signal A pin
const int encoder1PinB = 25; // Encoder 1 signal B pin
const int encoder2PinA = 33; // Encoder 2 signal A pin
const int encoder2PinB = 32; // Encoder 2 signal B pin

const int pulsesPerRevolution = 2048; //PPR for the AMT1120

const float link1Length = 107.0; // Length of link 1 in mm
const float link2Length = 107.0; // Length of link 2 in mm

volatile int encoderPosition1 = 0;
volatile int encoderPosition2 = 0;

// Desired end effector positions (x, y) 
const float x_desired[] = {135,-135}; 
const float y_desired[] = {10,-10};   

float kp1 = 0.596; // Proportional gain motor 1
float kp2 = 0.356; // Proportional gain motor 2
float kd = -0.005; // Derivative gain

float previousError1 = 0.0; // Initialize previous error for X
float previousError2 = 0.0; // Initialize previous error for Y

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
  pinMode(M2_D1, OUTPUT);
  pinMode(M1_PwM, OUTPUT);
  pinMode(M2_PwM, OUTPUT);
}

void loop() {
  // Calculate the current end effector position (x_current, y_current) using forward kinematics
  float theta1 = radians(static_cast<float>(encoderPosition1) / pulsesPerRevolution * 360.0);
  float theta2 = radians(static_cast<float>(encoderPosition2) / pulsesPerRevolution * 360.0);
  
  // Calculate joint angles using inverse kinematics
  float x_current = link1Length * cos(theta1) + link2Length * cos(theta1 + theta2);
  float y_current = link1Length * sin(theta1) + link2Length * sin(theta1 + theta2);

  // Calculate the error between the current and desired positions
  float error_x = x_desired[currentPositionIndex] - x_current;
  float error_y = y_desired[currentPositionIndex] - y_current;

  // Calculate the Jacobian matrix
  float J[2][2];
  J[0][0] = -link1Length * sin(theta1) - link2Length * sin(theta1 + theta2);
  J[0][1] = -link2Length * sin(theta1 + theta2);
  J[1][0] = link1Length * cos(theta1) + link2Length * cos(theta1 + theta2);
  J[1][1] = link2Length * cos(theta1 + theta2);

  // Calculate the pseudo-inverse of the Jacobian matrix
  float J_inv[2][2];
  float det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
  J_inv[0][0] = J[1][1] / det;
  J_inv[0][1] = -J[0][1] / det;
  J_inv[1][0] = -J[1][0] / det;
  J_inv[1][1] = J[0][0] / det;

  // Calculate the desired end effector velocity
  float endEffectorVelocity[2];
  endEffectorVelocity[0] = kp1 * error_x;
  endEffectorVelocity[1] = kp2 * error_y;

  // Calculate the joint velocities using the Jacobian transpose
  float jointVelocity[2];
  jointVelocity[0] = J_inv[0][0] * endEffectorVelocity[0] + J_inv[0][1] * endEffectorVelocity[1];
  jointVelocity[1] = J_inv[1][0] * endEffectorVelocity[0] + J_inv[1][1] * endEffectorVelocity[1];

  // Update encoder positions based on joint velocities
  encoderPosition1 += static_cast<int>(degrees(jointVelocity[0]) / 360.0 * pulsesPerRevolution);
  encoderPosition2 += static_cast<int>(degrees(jointVelocity[1]) / 360.0 * pulsesPerRevolution);

  // Map encoder positions to motor speeds (adjust these mappings as needed)
  int motorSpeed1 = map(encoderPosition1, -pulsesPerRevolution, pulsesPerRevolution, -255, 255);
  int motorSpeed2 = map(encoderPosition2, -pulsesPerRevolution, pulsesPerRevolution, -255, 255);

  // Move the motors based on motor speeds
  digitalWrite(M1_D1, motorSpeed1 > 0 ? HIGH : LOW);
  digitalWrite(M2_D1, motorSpeed2 > 0 ? HIGH : LOW);
  analogWrite(M1_PwM, abs(motorSpeed1));
  analogWrite(M2_PwM, abs(motorSpeed2));

  // Check if the end effector is close to the desired position
  if (abs(error_x) < 5.0 && abs(error_y) < 5.0) {
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
  delay(1000);
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
