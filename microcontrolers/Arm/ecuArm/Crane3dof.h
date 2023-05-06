#include "motorsWencoder.h"
#ifndef CRANE3DOF_H
#define CRANE3DOF_H
class Crane3dof{
  private:

    float origin2water;
    DCmotor_Encoder first_motor;
    DCmotor_Encoder second_motor;
    DCmotor_Encoder third_motor;


    float x_desired, y_desired, z_desired; /*Set thorugh i2c communication intereruption*/
    
    /*Encoder resolutions in pulses per revolution*/
    unsigned int joint1_encoder_resolution;
    unsigned int joint2_encoder_resolution ;
    unsigned int joint3_encoder_resolution;

   /**/
    float theta,radius,z_height;

  public:
  Crane3dof ();
    /*Assumes corrdinate frame located in the center of the robotic arm*/
  void inverse_kinematics(float x, float y, float z);
  /*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
  void setTargetJoints();
  void reachPosition(float deltaTime);
  void moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution);

  void updateMotors(unsigned char index);
  // Auxiliary testing methods
  void printMotorGains();
};


#endif