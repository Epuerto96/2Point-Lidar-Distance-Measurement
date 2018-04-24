#include <Arduino.h>
#include <Wire.h>
#include <LIDARLite.h>

int i; //variable for the counter in the for loops
int dirpin = 5; //variable that holds the pin number of the direction pin for the easy driver
int steppin = 4;
int dirpin2 = 3; //variable that holds the pin number of the direction pin for the easy driver
int steppin2 = 2;
int SW_pin = 12;
boolean stepDirection = true;
boolean stepDirection2 = true;

const int X_pin = 0; // analog pin connected to X output
const int Y_pin = 1; // analog pin connected to Y output
int xValue;
int yValue;
double stepAngle = .225;
double xAngle;
double yAngle;

boolean startPoint = true;
double startD;
double endD;

LIDARLite myLidarLite;


/**
 * distanceCalc() Used for calculation of length from lidar with gimbal
 * @param  thetaX        x angle of rotation from starting vector
 * @param  thetaY        y angle of rotation from starting vector
 * @param  startDistance the length gathered for the starting vector
 * @param  endDistance   the legth gathered for the ending vector
 * @return               returns length as a double in meters
 */
double distanceCalc(double thetaX, double thetaY, double startDistance, double endDistance){
  double radX = thetaX * 1000 / 57296; //Converts intial input from degrees to radians
  double radY = thetaY * 1000 / 57296;
  double radFinal = 2*acos(cos(radX / 2) * cos(radY / 2)); // quaternion conversion and mulpliplication results in single 2d angle
//  double thetaFinal = radFinal * 57296 / 1000;
//  Serial.println(radFinal);
  double length = sqrt(sq(startDistance) + sq(endDistance) - 2 * startDistance * endDistance * (cos(radFinal))); //Law of sins calculation
  return length; //
}


/**
 * Logs steps to determine angle rotated
 * @param Rotation takes in boolean function to determine CW or CCW
 */
void updatexAngle(boolean Rotation){
  if(Rotation == true){
    xAngle = xAngle + stepAngle;
    if(xAngle >= 360){
      xAngle = xAngle - 360;
      }
    }
  if(Rotation == false){
    xAngle = xAngle - stepAngle;
    if(xAngle <= -360){
      xAngle = xAngle + 360;
      }
    }
  }
void updateyAngle(boolean Rotation2){
  if(Rotation2 == true){
    yAngle = yAngle + stepAngle;
    if(yAngle >= 360){
      yAngle = yAngle - 360;
      }
    }
  if(Rotation2 == false){
    yAngle = yAngle - stepAngle;
    if(yAngle <= -360){
      yAngle = yAngle + 360;
      }
    }
  }


  void updateJoy(){
    xValue = analogRead(X_pin);
//    Serial.println(xValue);
    yValue = analogRead(Y_pin);
//    Serial.println(yValue);
    }


  void servoRotate(){
     // Controls Clockwise Counter Clockwise
    while(xValue >=545 || xValue <= 515){
      if (xValue > 545){
        stepDirection = false;
        }
      if(xValue < 515){
        stepDirection = true;
        }
      digitalWrite(dirpin, stepDirection);
      digitalWrite(steppin, LOW);
      digitalWrite(steppin, HIGH);
      delayMicroseconds(15000);  // Higher Number, Slower Rotation
      updateJoy();
      updatexAngle(stepDirection);
      }
    while(yValue >=535 || yValue <= 505){
      if (yValue > 535){
        stepDirection2 = false;
        }
      if(yValue < 515){
        stepDirection2 = true;
        }
      digitalWrite(dirpin2, stepDirection2);
      digitalWrite(steppin2, LOW);
      digitalWrite(steppin2, HIGH);
      delayMicroseconds(15000);  // Higher Number, Slower Rotation
      updateJoy();
      updateyAngle(stepDirection2);
      }
  }


double lidarAverage(){
  double sum = 0;
  double count = 0;
  for (int i=0; i != 100; i++){
  sum = sum + myLidarLite.distance();
//  Serial.println(sum);
//  delay(100);
  count++;
  }
  double average = sum/count;
  return average;
}


void setup() {
  Serial.begin(115200); //opens a serial connection at 115200 bps
  pinMode(dirpin, OUTPUT); //declares pin 3 as an output for direction control
  pinMode(steppin, OUTPUT); // Stepping Pin
  pinMode(dirpin2, OUTPUT);
  pinMode(steppin2, OUTPUT);
  pinMode(SW_pin, INPUT);
  digitalWrite(SW_pin, HIGH);
  myLidarLite.begin(0, true);
  myLidarLite.configure(0);

}

void loop() {
  updateJoy();
  servoRotate();
//  Serial.println(lidarAverage());
  if(digitalRead(SW_pin) == LOW && startPoint == true){
    startD = lidarAverage();
    startPoint = false;
    Serial.print("Start Point Distance: ");
    Serial.println(startD);
    delay(500);

  }
  if (digitalRead(SW_pin) == LOW && startPoint == false) {
    endD = lidarAverage();
    startPoint = true;
    Serial.print("End Point Distance: ");
    Serial.println(endD);
    delay(500);
    Serial.print("Distance Between Points: ");
    Serial.println(distanceCalc(xAngle,yAngle,startD,endD));
    xAngle = 0;
    yAngle = 0;

  }

}
