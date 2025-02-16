#include <Servo.h>
const int TRIG = 6;
const int ECHO = 5;
Servo myServo;

long distance_L;
long distance_R;

int maxDistanceFromObstacles = 40; //ADDED THIS

void setup() {
  // put your setup code here, to run once:
  
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  myServo.attach(11);
   Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  long distanceFromObstacles = ultrasonicRead();
  Serial.println(distanceFromObstacles);
  if(distanceFromObstacles < maxDistanceFromObstacles){
    checkSide();
  }
 
}

long ultrasonicRead(){
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long time = pulseIn (ECHO, HIGH);
  return time * 0.0343 / 2;
}

void checkSide()
{
for (int angle = 70; angle <= 140; angle += 5)  {
    // servoPulse(servo, angle);  
    myServo.write(angle);
  }
  delay(1000);
  distance_R = ultrasonicRead();
  Serial.print("D R=");
  Serial.println(distance_R);
  delay(100);

  for (int angle = 140; angle >= 0; angle -= 5)  {
    // servoPulse(servo, angle); 
    myServo.write(angle);
  }
  delay(1000);
  distance_L = ultrasonicRead();
  Serial.print("D L=");
  Serial.println(distance_L);
  delay(100);

 for (int angle = 0; angle <= 70; angle += 5)  {
    // servoPulse(servo, angle);  
    myServo.write(angle);
  }

  delay(300);
  // compareDistance();
}
