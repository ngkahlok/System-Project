#include <Servo.h>
#include <Wire.h>
#include <Stepper.h>
#include <LiquidCrystal_I2C.h> //LCD
#include <Adafruit_PWMServoDriver.h> //Servo Motor Drvier
#include "Adafruit_TCS34725.h" //Colour Sensor

//Define Colour Sensor 
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); 

// Create PCA9685 object at default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

//Define SDA/SCL Multiplexer
#define TCA_ADDRESS 0x70 

//Define for I2C LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); 

bool compartmentFilled[6] = {false, false, false, false, false, false};
//red, blue, yellow, white, black, baseReached

// Colour Sensor Variable
char currentColour;
int previousColourIndex = 0;

//Stepper Motor
#define STEPS_PER_REV 2048  // Full steps for 28BYJ-48 stepper
#define SECTIONS 6  // Number of sections on the plate
//Need to change
Stepper stepperMotor(STEPS_PER_REV, 22, 23, 24, 25);  // IN1-IN4 connected to 22, 23, 24, 25

//Robotic Arm
// Define min & max pulse lengths for servos
#define SERVOMIN 150  // Minimum pulse length (0°)
#define SERVOMAX 590  // Maximum pulse length (180°)
// Servo channels (adjust based on your setup)
#define BASE_SERVO 0   // Base rotation
#define LEFT_SERVO 1   // Left arm movement
#define RIGHT_SERVO 2  // Right arm movement
#define CLAW_SERVO 3   // Claw movement
//Previous Servo Position
int previousAngleBase = 0;
int previousAngleLeft = 0; //15 move up - 80 move down
int previousAngleRight = 0; //0 move back - 90 move front 
int previousAngleClaw = 0; //80 close - 150 open

//DC Motor Driver
const int ENA = 7;
const int IN1 = 6;
const int IN2 = 5;
const int IN3 = 4;
const int IN4 = 3;
const int ENB = 2;

//Ultrasonic Sensor
const int TRIG = 11;
const int ECHO = 12;

//Ultrasonic's Servo
Servo myServo;

//Buzzer
const int Buzz = 14;

//IR Sensors
const int IrLeftSensor = 8;
const int IrRightSensor = 9;
const int IrCenterSensor = 10;

// PID constants
float Kp = 1;   // Proportional gain
float Ki = 0.0;   // Integral gain (set to 0 initially)
float Kd = 0.5;   // Derivative gain

// Variables for PID control
float error = 0;              // Current error
float lastError = 0;          // Previous error (for derivative)
float integral = 0;           // Cumulative error (for integral)

// Motor speed variables
int baseSpeed = 75;          // Base speed of the motors
int leftMotorSpeed;
int rightMotorSpeed;

long distance_L;
long distance_R;

//Obstacles Avoidance 
int maxDistanceFromObstacles = 25; //ADDED THIS

int initialStep = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  //IR Sensor
  pinMode(IrLeftSensor, INPUT);
  pinMode(IrRightSensor, INPUT);
  pinMode(IrCenterSensor, INPUT);
  //DC Motor Driver
  pinMode(IN1, OUTPUT);    
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  //Ultrasonic Sensor
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //Initialise PWM
  // tcaSelect(1);
  // pwm.begin();
  // pwm.setPWMFreq(50);  // Set frequency to 50 Hz (for servos)
  delay(200);

   //LCD
  tcaSelect(2);
  lcd.init();       // Initialize the LCD
  lcd.backlight();  // Turn on the backlight
  //Initialising
  // Serial.println("Initializing Robot");
  delay(200);
  lcdPrint("Initializing");
  
  ultrasonicRead();
  //Ultrasonic's Servo
  myServo.attach(13);
  //Buzzer
  pinMode(Buzz,OUTPUT);
  delay(200);
  //Colour Sensor
  tcaSelect(0);
  tcs.begin();
  if (!tcs.begin())
  {
    Serial.println("TS34725 not found. Check wiring!");
  }else{
    Serial.println("TCS34725 Ready!");
  }
  //Stepper Motor
  stepperMotor.setSpeed(8);
  tcaSelect(1);
  pwm.begin();
  pwm.setPWMFreq(50);  // Set frequency to 50 Hz (for servos)
  // delay(100);
  delay(2000);

  //Robot Arm
  // Move all servos to their initial positions
  moveWithDelay(BASE_SERVO, 0);
  delay(1000);
  moveWithDelay(RIGHT_SERVO, 0); // Move right servo to 135°
  delay(1000);
  moveWithDelay(LEFT_SERVO, 50);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(CLAW_SERVO, 165);   // Open claw slightly
  delay(1000);

  //Done Setting Up
  Serial.println("Setup Completed");
  lcdPrint("Setup Completed");
  delay(1000);
}

void loop() {
  tcaSelect(2);
  lcdPrint("ARMEX 01");

  int left = digitalRead(IrLeftSensor);
  int center = digitalRead(IrCenterSensor);
  int right = digitalRead(IrRightSensor);

  // ColourSensing();
  long distanceFromObstacles = ultrasonicRead();
  PID(left,center,right);

  //Main Flow 
  if((distanceFromObstacles < maxDistanceFromObstacles) && (distanceFromObstacles!= 0)){
    stopMotors();
    tcaSelect(2);
    // lcdPrint(distanceFromObstacles);
    // delay(2000);
    lcdPrint("Obstacles Detected");
    Buzzer(); //Horn
    lcdPrint("Please Move");
    distanceFromObstacles = ultrasonicRead();
    if(distanceFromObstacles > 25){
      moveForward(70);
    }
    else{
      delay(3000);
      checkSide();
    }
  }
  else{
    if (left == 0 && center == 1 && right == 0) {
        if(initialStep==0){
          moveForward(75);
          initialStep++;
        }else{
           moveForward(); // Robot is centered on the line
        }
    } else if (left == 1 && center == 1 && right == 0) {
        turnLeft(); // Slightly right of the line
    } else if (left == 0 && center == 1 && right == 1) {
        turnRight(); // Slightly left of the line
    } else if (left == 1 && center == 0 && right == 0) {
        turnLeft(); // Far right
    } else if (left == 0 && center == 0 && right == 1) {
        turnRight(); // Far left
    } else if (left == 0 && center == 0 && right == 0) {
        reverse();
    } else if(left == 1 && center == 1 && right == 1){
      stopMotors();
      delay(3000);
      tcaSelect(0);

      char firstSense, secondSense;

      do {
          firstSense = ColourSensing();
          delay(1000);
          secondSense = ColourSensing();
      } while (firstSense != secondSense);

      currentColour = secondSense;
      // colourDics(currentColour);
      
      if((currentColour != 'U')&&(currentColour != 'G')){ //No bin collected
        // tcaSelect(1);
        colourDics(currentColour);
        delay(100);
        int colour;
        switch(currentColour){
          case 'R':
            colour = 0;
            break;
          case 'B':
            colour = 1;
            break;
          case 'Y':
            colour = 2;
            break;
          case 'W':
            colour = 3;
            break;
          case 'H':
            colour = 4;
            break;
        }
        if(compartmentFilled[colour]!=true){
          collectBin();
          binCollected(currentColour); //set compartment filled to true
        }
      }
      else if(currentColour == 'G'){
        for(int i = 0; i < 5; i++){
          if(compartmentFilled[i]==true){
            switch(i){
              case 0:
                colourDicsForDrop('R');
                break;
              case 1:
                colourDicsForDrop('B');
                break;
              case 2:
                colourDicsForDrop('Y');
                break;
              case 3:
                colourDicsForDrop('W');
                break;
              case 4:
                colourDicsForDrop('H');
                break;
              default:
                return; 
            }
            delay(100);
            disposeBin(); //Need to rotate the compartment too & set the values to true when it is picked up
            compartmentFilled[i] = false; //Set back to false after droped off
          }
        }
      }
      delay(1000);
      do{
        moveForward();
        delay(10); 
      } while(digitalRead(IrLeftSensor) == 1 && 
         digitalRead(IrCenterSensor) == 1 && 
         digitalRead(IrRightSensor) == 1);
      
    }
  }
}

//Line Following
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
  
  // analogWrite(ENA, rightMotorSpeed);
  // analogWrite(ENB, leftMotorSpeed);
  analogWrite(ENA, 65);
  analogWrite(ENB, 65);
  delay(10);
}

void moveForward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  // analogWrite(ENA, rightMotorSpeed);
  // analogWrite(ENB, leftMotorSpeed);
  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
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

void turnLeftAvoid(int left, int right) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  
  analogWrite(ENA, right);
  analogWrite(ENB, left);
  delay(10);
}

void turnRightAvoid(int left, int right) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  
  analogWrite(ENA, right);
  analogWrite(ENB, left);
  delay(10);
}

void stopMotors() {
  // lcdPrint("Motor Stop");
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  delay(10);
}

//Obstacles Avoidance
//Ultrasonic Sensor 
long ultrasonicRead(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long time = pulseIn (ECHO, HIGH);
  return time * 0.0343 / 2;
}

void compareDistance(){
  if(distance_L > distance_R){
    AvoidanceLeft();
  } 
  else if(distance_R > distance_L){
    AvoidanceRight();
  }
}

void checkSide(){
  // Servo angle 0 = right, angle 90 = middle ,angle 180 = left
  stopMotors();
  delay(100);

 for (int angle = 90; angle <= 140; angle += 5)  {
     
    myServo.write(angle);
    delay(50);
  }
  delay(1000);
  distance_L = ultrasonicRead();
  Serial.print("D L=");
  Serial.println(distance_L);
  delay(100);

  for (int angle = 140; angle >= 20; angle -= 5)  { 
    myServo.write(angle);
    delay(50);
  }
  delay(1000);
  distance_R = ultrasonicRead();
  Serial.print("D R=");
  Serial.println(distance_R);
  delay(100);

 for (int angle = 20; angle <= 90; angle += 5)  {
    myServo.write(angle);
    delay(50);
  }
  delay(300);
  compareDistance();
}

//Buzzer
void Buzzer(){
   tone(Buzz, 1000);
   delay(1000);
   tone(Buzz, 1500);
   delay(1000);
   noTone(Buzz);
   delay(500);
}

// SDA/SCL Multiplexer
void tcaSelect(uint8_t channel) { //Selecting channel for multiplexer according to functions
  if (channel > 7) return;
  Wire.beginTransmission(TCA_ADDRESS);
  Wire.write(1 << channel);  // Select the desired channel
  Wire.endTransmission();
}

//LCD Printing
void lcdPrint(const char* sentence){
  tcaSelect(2);
  lcd.clear();
  lcd.setCursor(0, 0);  // Set cursor to column 0, row 0
  lcd.print(sentence);
}

//Robotic Arm
// Convert an angle (0-180°) to a pulse width
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// // Function to move a servo to a specific angle
void moveServo(int servoChannel, int angle) {
  int pulse = angleToPulse(angle);
  pwm.setPWM(servoChannel, 0, pulse);
}

void moveWithDelay(int servoChannel, int angle){
  delay(100);
  tcaSelect(1);
  int previousAngle = 0;

  switch(servoChannel){
    case BASE_SERVO:
      previousAngle = previousAngleBase;
      break;
    case LEFT_SERVO:
      previousAngle = previousAngleLeft;
      break;
    case RIGHT_SERVO:
      previousAngle = previousAngleRight;
      break;
    case CLAW_SERVO:
      previousAngle = previousAngleClaw;
      break;
  }

  if(previousAngle >= angle){
    for(int i = previousAngle; i >= angle; i-=5){
    moveServo(servoChannel, i); 
    delay(50);
    }
  }
  else if(previousAngle < angle){
    for(int i = previousAngle; i <= angle; i+=5){
    moveServo(servoChannel, i); 
    delay(50);
    }
  }

  // previousAngle = angle;
    switch(servoChannel){
    case BASE_SERVO:
      previousAngleBase = angle;
      break;
    case LEFT_SERVO:
      previousAngleLeft = angle;
      break;
    case RIGHT_SERVO:
      previousAngleRight = angle;
      break;
    case CLAW_SERVO:
      previousAngleClaw = angle;
      break;
  }
}

void disposeBin(){
  lcdPrint("Disposing Bin");
  // tcaSelect(1);
  // moveWithDelay(LEFT_SERVO, 60); // Move right servo to 135°
  // delay(1000);
  // moveWithDelay(RIGHT_SERVO, 0); // Move right servo to 135°
  // delay(1000);
  moveWithDelay(BASE_SERVO, 95);
  delay(1000);
  moveWithDelay(RIGHT_SERVO, -40);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(LEFT_SERVO, 29);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(RIGHT_SERVO, 13);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(CLAW_SERVO, 90);   // Open claw slightly
  delay(1000);
  moveWithDelay(LEFT_SERVO, 60);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(BASE_SERVO, 0);
  delay(1000); //
  moveWithDelay(RIGHT_SERVO, 20);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(LEFT_SERVO, 20);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(CLAW_SERVO, 165);   // Open claw slightly
  delay(1000);
  moveWithDelay(LEFT_SERVO, 60); // Move right servo to 135°
  delay(1000);
  moveWithDelay(RIGHT_SERVO, 0); // Move right servo to 135°
  delay(1000);
}

void collectBin(){
  lcdPrint("Collecting Bin");
  // tcaSelect(1);
  moveWithDelay(LEFT_SERVO, 15);  // Move left servo to 45°
  delay(1000);
  moveWithDelay(RIGHT_SERVO, 40); // Move right servo to 135°
  delay(1000);
   
  moveWithDelay(CLAW_SERVO, 80);   // Open claw slightly
  delay(1000);
  
  moveWithDelay(LEFT_SERVO, 60); // Move right servo to 135°
  delay(1000);
  moveWithDelay(RIGHT_SERVO, 0); // Move right servo to 135°
  delay(1000);

  moveWithDelay(BASE_SERVO, 95);
  delay(1000);
  
  moveWithDelay(CLAW_SERVO, 165);   // Open claw slightly
  delay(1000);
  moveWithDelay(BASE_SERVO, 0);
  delay(1000);


  lcdPrint("Bin Collected");
}

//Bin Compartment
void colourDics(char colour){
  int index = 0;
  int sectionDifference = 0;
  switch(colour){
    case 'R':
      index = 1;
      break;
    case 'B':
      index = 2;
      break;
    case 'Y':
      index = 3;
      break;
    case 'W':
      index = 4;
      break;
    case 'H':
      index = 5;
      break;
    default:
      index = 6;
  }

  if(index > previousColourIndex){ //turn clockwise
  sectionDifference = index - previousColourIndex;
  }
  else if(index < previousColourIndex){
  sectionDifference = 6 - abs(index - previousColourIndex);
  }
  
  float stepsPerSection = STEPS_PER_REV / SECTIONS;  // Steps for each 1/6 turn
  // int delayTime = 10000;  // Time to wait at each section (2 seconds)

  if(index!=6){
    stepperMotor.step(ceil(stepsPerSection * sectionDifference)+45); //Given that it is starting fro initial position
  }else{
    stepperMotor.step(ceil(stepsPerSection * sectionDifference));
  }

  previousColourIndex = index;
}

void colourDicsForDrop(char colour){
  int index = 0;
  int sectionDifference = 0;
  switch(colour){
    case 'R':
      index = 1;
      break;
    case 'B':
      index = 2;
      break;
    case 'Y':
      index = 3;
      break;
    case 'W':
      index = 4;
      break;
    case 'H':
      index = 5;
      break;
    default:
      index = 6;
  }

  if(index > previousColourIndex){ //turn clockwise
  sectionDifference = index - previousColourIndex;
  }
  else if(index < previousColourIndex){
  sectionDifference = 6 - abs(index - previousColourIndex);
  }
  
  float stepsPerSection = STEPS_PER_REV / SECTIONS;  // Steps for each 1/6 turn
  // int delayTime = 10000;  // Time to wait at each section (2 seconds)
  if(index!=6){
    stepperMotor.step(ceil(stepsPerSection * sectionDifference)+55); //Given that it is starting fro initial position
  }else{
    stepperMotor.step(ceil(stepsPerSection * sectionDifference));
  }
  
  previousColourIndex = index;
}

void binCollected(char colour){
  switch(colour){
    case 'R':
      compartmentFilled[0]=true;
      break;
    case 'B':
      compartmentFilled[1]=true;
      break;
    case 'Y':
      compartmentFilled[2]=true;
      break;
    case 'W':
      compartmentFilled[3]=true;
      break;
    case 'H':
      compartmentFilled[4]=true;
      break;
    case 'G':
      compartmentFilled[5]=true; //reached base
      break;
    default:
      return; 
  }
}

//Colour Sensor
char ColourSensing(){
  tcaSelect(0);
  uint16_t red, green, blue, clear; // Variables for color values
  tcs.getRawData(&red, &green, &blue, &clear); // Read sensor data

  float redRatio = (float)red / clear;
  float greenRatio = (float)green / clear;
  float blueRatio = (float)blue / clear;


  Serial.print("Red: "); Serial.print(redRatio, 2);
  Serial.print(" Green: "); Serial.print(greenRatio, 2);
  Serial.print(" Blue: "); Serial.println(blueRatio, 2);

  // Determine the color
  if (redRatio > 0.4 && greenRatio < 0.3 && blueRatio < 0.3) {
    Serial.println("Detected: RED");
    lcdPrint("Detected: RED");
    return 'R';
  } 
  else if (redRatio < 0.3 && greenRatio > 0.3 && blueRatio > 0.4) {
    Serial.println("Detected: BLUE");
    lcdPrint("Detected: BLUE");
    return 'B';
  } 
  else if (redRatio > 0.3 && greenRatio > 0.3 && blueRatio < 0.2) {
    Serial.println("Detected: YELLOW");
    lcdPrint("Detected: YELLOW");
    return 'Y';
  } 
  else if (redRatio < 0.4 && greenRatio > 0.4 && blueRatio < 0.3) {
    Serial.println("Detected: GREEN");
    // lcdPrint("Detected: GREEN");
    lcdPrint("Reached Base");
    return 'G';
  }else if (redRatio > 0.3 && greenRatio > 0.3 && blueRatio > 0.3) {
    if(clear > 2500){ // Low light means black
      Serial.println("Detected: WHITE");
      lcdPrint("Detected: WHITE");
      return 'W';
    } 
    else {
      Serial.println("Detected: BLACK");
      lcdPrint("Detected: BLACK");
      return 'H';
    }
  }
  else{
    Serial.println("Detected: UNKNOWN COLOUR");
    lcdPrint("No Bin Detected");
    // lcdPrint("No Colour Detected");
    return 'U';
  }
}

void AvoidanceRight(){
    for(int left = 0; left < 1; left ++){
      turnRightAvoid(95,95);
      delay(500);
    }
    for(int left = 0; left < 3; left ++){
      moveForward(68);
      delay(500);
    }
     for(int left = 0; left < 1; left ++){
      turnLeftAvoid(95,95);
      delay(500);
    }
     for(int left = 0; left < 2; left ++){
      moveForward();
      delay(500);
    }
    for(int left = 0; left < 1; left ++){
      turnLeftAvoid(95,95);
      delay(500);
    }

    // for(int left = 0; left < 2; left ++){
    //   moveForward();
    //   delay(500);
    // }

    do{
       moveForward();
       delay(10); 
    }while(digitalRead(IrLeftSensor) == 0 && 
       digitalRead(IrCenterSensor) == 0 && 
      digitalRead(IrRightSensor) == 0);

    // for(int left = 0; left < 3; left ++){
    //   turnRightAvoid(85,85);
    //   delay(500);
    // }
    do{
        turnRightAvoid(90,90);
       delay(10); 
    }while((digitalRead(IrLeftSensor) == 1 && 
       digitalRead(IrCenterSensor) == 1 && 
      digitalRead(IrRightSensor) == 0)||(digitalRead(IrLeftSensor) == 1 && 
       digitalRead(IrCenterSensor) == 1 && 
      digitalRead(IrRightSensor) == 1)||(digitalRead(IrLeftSensor) == 1 && 
       digitalRead(IrCenterSensor) == 0 && 
      digitalRead(IrRightSensor) == 0));

    // for(int left = 0; left < 1; left ++){
    //   turnLeftAvoid(85,85);
    //   delay(500);
    // }
    
}

void AvoidanceLeft(){
    for(int left = 0; left < 1; left ++){
      turnLeftAvoid(95,95);
      delay(500);
    }
    for(int left = 0; left < 3; left ++){
      moveForward(68);
      delay(500);
    }
     for(int left = 0; left < 1; left ++){
      turnRightAvoid(95,95);
      delay(500);
    }
     for(int left = 0; left < 2; left ++){
      moveForward();
      delay(500);
    }
    for(int left = 0; left < 1; left ++){
      turnRightAvoid(95,95);
      delay(500);
    }
    // for(int left = 0; left < 2; left ++){
    //   moveForward();
    //   delay(500);
    // }
    do{
       moveForward();
       delay(10); 
    }while(digitalRead(IrLeftSensor) == 0 && 
       digitalRead(IrCenterSensor) == 0 && 
      digitalRead(IrRightSensor) == 0);

    do{
        turnLeftAvoid(90,90);
       delay(10); 
    }while((digitalRead(IrLeftSensor) == 0 && 
       digitalRead(IrCenterSensor) == 1 && 
      digitalRead(IrRightSensor) == 1)||(digitalRead(IrLeftSensor) == 1 && 
       digitalRead(IrCenterSensor) == 1 && 
      digitalRead(IrRightSensor) == 1)||(digitalRead(IrLeftSensor) == 0 && 
       digitalRead(IrCenterSensor) == 0 && 
      digitalRead(IrRightSensor) == 1));
    // for(int left = 0; left < 3; left ++){
    //   turnLeftAvoid(85,85);
      
    //   delay(500);
    // }
    // for(int left = 0; left < 1; left ++){
    //   turnRightAvoid(85,85);
    //   delay(500);
    // }
    
}