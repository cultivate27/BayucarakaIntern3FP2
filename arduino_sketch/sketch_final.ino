#include <ros.h>
#include <std_msgs/String.h>
#include <AccelStepper.h>
#include <Servo.h>

#define dirPinX 5
#define stepPinX 2
#define dirPinY 6
#define stepPinY 3

#define servoPin 11

AccelStepper stepperX(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper stepperY(AccelStepper::DRIVER, stepPinY, dirPinY);
Servo myServo;

ros::NodeHandle nh;

int x;
int y;

void callback(const std_msgs::String &cmd) {

  String data = String(cmd.data);

  int comma1 = data.indexOf(',');

  x = data.substring(0, comma1).toInt();
  y = data.substring(comma1 + 1).toInt();

  Serial.print("X: ");
  Serial.println(x); 
  Serial.print("Y: ");
  Serial.println(y); 
}

ros::Subscriber<std_msgs::String> sub("stepper_commands", &callback);

void setup() {
  Serial.begin(4800);
  
  nh.initNode();
  nh.subscribe(sub);
  stepperX.setMaxSpeed(3000);
  stepperX.setAcceleration(4000);
  stepperY.setMaxSpeed(3000);
  stepperY.setAcceleration(4000);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
   

  myServo.attach(servoPin);
  myServo.write(0);
}

void loop() {
  nh.spinOnce();

  stepperX.move(x);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  stepperY.move(y);
  while (stepperY.distanceToGo() != 0) {
    stepperY.run();
  }

  delay(5000);

  stepperY.move(-y);
  while (stepperY.distanceToGo() != 0) {
    stepperY.run();
  }

  stepperX.move(-x);
  while (stepperX.distanceToGo() != 0) {
    stepperX.run();
  }

  delay(5000);
}