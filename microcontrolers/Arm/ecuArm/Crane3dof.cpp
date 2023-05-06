
#include "joint1_config.h"
#include "joint2_config.h"
#include "joint3_config.h"
#include "Crane3dof.h"
#include "math.h"
#include "Arduino.h"
#define PI                          3.14159265

struct MotorEncoderParams joint1 = 
{
    ENCODER_A_1,
    ENCODER_B_1,
    POSITIVE_DIR_PIN_1,
    NEGATIVE_DIR_PIN_1,
    PWM_PIN_1,
    MIN_PWM_SIGNAL,
    MAX_PWM_SIGNAL,
    KP_1,
    KD_1,
    KI_1,
    JOINT1_LOW_LIMIT_HW,        
    JOINT1_HIGH_LIMIT_HW, 
    JOINT1_LOW_LIMIT_SW,                  
    JOINT1_HIGH_LIMIT_SW  
};

struct MotorEncoderParams joint2 = 
{
    ENCODER_A_2,
    ENCODER_B_2,
    POSITIVE_DIR_PIN_2,
    NEGATIVE_DIR_PIN_2,
    PWM_PIN_2,
    MIN_PWM_SIGNAL,
    MAX_PWM_SIGNAL,
    KP_2,
    KD_2,
    KI_2,
    JOINT2_LOW_LIMIT_HW,        
    JOINT2_HIGH_LIMIT_HW, 
    JOINT2_LOW_LIMIT_SW,                  
    JOINT2_HIGH_LIMIT_SW  
};

struct MotorEncoderParams joint3 = 
{
    ENCODER_A_3,
    ENCODER_B_3,
    POSITIVE_DIR_PIN_3,
    NEGATIVE_DIR_PIN_3,
    PWM_PIN_3,
    MIN_PWM_SIGNAL,
    MAX_PWM_SIGNAL,
    KP_3,
    KD_3,
    KI_3,
    JOINT3_LOW_LIMIT_HW,        
    JOINT3_HIGH_LIMIT_HW, 
    JOINT3_LOW_LIMIT_SW,                  
    JOINT3_HIGH_LIMIT_SW  
};




Crane3dof::Crane3dof ():first_motor(joint1),
second_motor(joint2),
third_motor(joint3){
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

