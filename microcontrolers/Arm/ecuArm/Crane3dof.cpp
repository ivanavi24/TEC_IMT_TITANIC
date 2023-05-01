
#include "joint1_config.h"
#include "joint2_config.h"
#include "joint3_config.h"
#include "Crane3dof.h"
#include "math.h"
#define PI                          3.14159265

Crane3dof::Crane3dof ():first_motor(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT1_LOW_LIMIT_HW,JOINT1_HIGH_LIMIT_HW,JOINT1_LOW_LIMIT_SW,JOINT1_HIGH_LIMIT_SW),
second_motor(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT2_LOW_LIMIT_HW,JOINT2_HIGH_LIMIT_HW,JOINT2_LOW_LIMIT_SW,JOINT2_HIGH_LIMIT_SW),
third_motor(KP_1,KI_1,KD_1,POSITIVE_DIR_PIN_1,NEGATIVE_DIR_PIN_1,JOINT3_LOW_LIMIT_HW,JOINT3_HIGH_LIMIT_HW,JOINT3_LOW_LIMIT_SW,JOINT3_HIGH_LIMIT_SW){
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
    

