const int ENA = 7;
const int IN1 = 6;
const int IN2 = 5;
const int IN3 = 4;
const int IN4 = 3;
const int ENB = 2;

const int IrLeftSensor = 8;
const int IrRightSensor = 9;
const int IrCenterSensor = 10;

// PID constants
float Kp = 1.2;   // Proportional gain
float Ki = 0.0;   // Integral gain (set to 0 initially)
float Kd = 0.6;   // Derivative gain

// Variables for PID control
float error = 0;              // Current error
float lastError = 0;          // Previous error (for derivative)
float integral = 0;           // Cumulative error (for integral)

// Motor speed variables
int baseSpeed = 90;          // Base speed of the motors
int leftMotorSpeed;
int rightMotorSpeed;

void setup() {
  // put your setup code here, to run once:
  pinMode(IrLeftSensor, INPUT);
  pinMode(IrRightSensor, INPUT);
  pinMode(IrCenterSensor, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  delay(1000);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int left = digitalRead(IrLeftSensor);
  int center = digitalRead(IrCenterSensor);
  int right = digitalRead(IrRightSensor);

  PID(left,center,right);
  
  if (left == 0 && center == 1 && right == 0) {
        moveForward(); // Robot is centered on the line
    } else if (left == 1 && center == 1 && right == 0) {
        turnLeft(); // Slightly right of the line
    } else if (left == 0 && center == 1 && right == 1) {
        turnRight(); // Slightly left of the line
    } else if (left == 1 && center == 0 && right == 0) {
        turnLeft(); // Far right
    } else if (left == 0 && center == 0 && right == 1) {
        turnRight(); // Far left
    } 
    else if (left == 0 && center == 0 && right == 0) {
        reverse();
    }
}

void PID(int left, int center, int right) {

    // Calculate error based on sensor readings
    if (left == 0 && center == 1 && right == 0) {
        error = 0; // Robot is centered on the line
    } else if (left == 1 && center == 1 && right == 0) {
        error = 1; // Slightly right of the line
    } else if (left == 0 && center == 1 && right == 1) {
        error = -1; // Slightly left of the line
    } else if (left == 1 && center == 0 && right == 0) {
        error = 2; // Far right
    } else if (left == 0 && center == 0 && right == 1) {
        error = -2; // Far left
    }

    // PID calculations
    float P = error;                        // Proportional term
    integral += error;                      // Integral term
    float I = integral;                     
    float D = error - lastError;            // Derivative term
    float correction = (Kp * P) + (Ki * I) + (Kd * D);

    // Update last error for next iteration
    lastError = error;

    // Calculate motor speeds
    leftMotorSpeed = baseSpeed - correction;
    rightMotorSpeed = baseSpeed + correction;

    // Constrain motor speeds to valid range (0-255)
    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    delay(10); // Short delay to stabilize readings
}

void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, rightMotorSpeed);
  analogWrite(ENB, leftMotorSpeed);
  delay(10);
}

void reverse() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  analogWrite(ENA, rightMotorSpeed);
  analogWrite(ENB, leftMotorSpeed);
  delay(10);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  analogWrite(ENA, rightMotorSpeed);
  analogWrite(ENB, leftMotorSpeed);
  delay(10);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, rightMotorSpeed);
  analogWrite(ENB, leftMotorSpeed);
  delay(10);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  delay(10);
}