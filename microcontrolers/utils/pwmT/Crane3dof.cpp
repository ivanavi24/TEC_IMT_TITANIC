
#include "joint1_config.h"
#include "joint2_config.h"
#include "joint3_config.h"
#include "Crane3dof.h"
#include "math.h"
#include "Arduino.h"
#define PI                          3.14159265

Crane3dof::Crane3dof ():first_motor(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,PWM_PIN_1,JOINT1_LOW_LIMIT_HW,JOINT1_HIGH_LIMIT_HW,JOINT1_LOW_LIMIT_SW,JOINT1_HIGH_LIMIT_SW,PWM_CHANNEL_1,PWM_FREQUENCY_1,PWM_RESOLUTION_1),
second_motor(KP_2,KI_2,KD_2,POSITIVE_DIR_PIN_2,NEGATIVE_DIR_PIN_2,PWM_PIN_2,JOINT2_LOW_LIMIT_HW,JOINT2_HIGH_LIMIT_HW,JOINT2_LOW_LIMIT_SW,JOINT2_HIGH_LIMIT_SW,PWM_CHANNEL_2,PWM_FREQUENCY_2,PWM_RESOLUTION_2),
third_motor(KP_3,KI_3,KD_3,POSITIVE_DIR_PIN_3,NEGATIVE_DIR_PIN_3,PWM_PIN_3,JOINT3_LOW_LIMIT_HW,JOINT3_HIGH_LIMIT_HW,JOINT3_LOW_LIMIT_SW,JOINT3_HIGH_LIMIT_SW,PWM_CHANNEL_3,PWM_FREQUENCY_3,PWM_RESOLUTION_3){
  origin2water=0;
  joint1_encoder_resolution = JOINT1_ENCODER_RESOLUTION;
  joint2_encoder_resolution = JOINT2_ENCODER_RESOLUTION;
  joint3_encoder_resolution = JOINT1_ENCODER_RESOLUTION;
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
  int pulsesJoint1 =(theta - ZERO_POS_1)* joint1_encoder_resolution/ (2* PI);   /*  pulses per revolution [pulses/rev]*  */ 
  int pulsesJoint2 =(radius - ZERO_POS_2)* joint2_encoder_resolution;   /*  pulses per revolution [pulses/rev]*  */ 
  int pulsesJoint3 =(z_height - ZERO_POS_3)* joint3_encoder_resolution;   /*  pulses per revolution [pulses/rev]*  */ 
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
  Serial.printf("Pwm motor 3 es: %d \n",dCycle);
  third_motor.moveMotor(dCycle);
}

