
#include "joint1_config.h"
#include "joint2_config.h"
#include "joint3_config.h"

#include "Crane3dof.h"
#include "math.h"
#include "Arduino.h"
#define PI                          3.14159265




Crane3dof::Crane3dof ():first_motor(joint1),
second_motor(joint2),
third_motor(joint3){
  origin2water=0;
  x_desired=0;
  y_desired=0;
  z_desired=0;
  theta=0;
  radius=0;
  z_height=0;

}
/*Assumes corrdinate frame located in the center of the robotic arm*/
void Crane3dof::inverse_kinematics(float x, float y, float z){
  theta =atan2(x,y);
  radius =sqrt(pow(x,2)+pow(y,2));
  z_height = z + origin2water; 
}

void Crane3dof::reachPosition(float deltaTime){
  
  first_motor.move2position(deltaTime);
  second_motor.move2position(deltaTime);
  third_motor.move2position(deltaTime);
  
}
/*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
void Crane3dof::setTargetJoints(){
  int pulsesJoint1 =(theta - ZERO_POS_1)/ (2* PI);   /*  pulses per revolution [pulses/rev]*  */ 
  int pulsesJoint2 =(radius - ZERO_POS_2)*REVOLUTIONS_PER_METER_2;   /*  revolutions per meter [rev/meters]*  */ 
  int pulsesJoint3 =(z_height - ZERO_POS_3)*REVOLUTIONS_PER_METER_3;   /*  revolutions per meter [pulses/rev]*  */ 
  first_motor.setJointDesired(pulsesJoint1);
  second_motor.setJointDesired(pulsesJoint2);
  third_motor.setJointDesired(pulsesJoint3);
}
void Crane3dof::moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution){
  int dCycle = (pwm[0]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 1 es: %d \n",dCycle);
  first_motor.moveMotor(dCycle);
  dCycle = (pwm[1]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 2 es: %d \n",dCycle);
  second_motor.moveMotor(dCycle);
  dCycle = (pwm[2]-minValue)/(maxValue-minValue)*pow(2,pwm_resolution);
  Serial.printf("Pwm motor 2 es: %d \n",dCycle);
  third_motor.moveMotor(dCycle); 
}
void Crane3dof::updateMotors(unsigned char index){
  switch (index)
  {
  case 1:
    first_motor.updateCurrentJoint();
    break;
  case 2:
    second_motor.updateCurrentJoint();
    break;
  case 3:
    third_motor.updateCurrentJoint();
    break;
  default:
    break;
  }
  
}

void Crane3dof::printMotorGains(){
  first_motor.displayGainValues();
  second_motor.displayGainValues();
  third_motor.displayGainValues();
}

