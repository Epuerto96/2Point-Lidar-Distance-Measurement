#include <Arduino.h>
#include <math.h>


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



void setup(){
  Serial.begin(115200);
}

void loop(){
  Serial.println(distanceCalc(-5,-10,5,5));
  delay(1000);
}
