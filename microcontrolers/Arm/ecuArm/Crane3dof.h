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
    

   /**/
    float theta,radius,z_height;
    

  public:
  Crane3dof ();
    /*Assumes corrdinate frame located in the center of the robotic arm*/
  /*Getters methods*/
  DCmotor_Encoder get_first_motor();
  DCmotor_Encoder get_second_motor();
  DCmotor_Encoder get_third_motor();

  /*Setter methods*/
  void setTargetRPM(unsigned int index);
  void setTargetAngle(unsigned int index);

  /*Inverse kinematic calculate */
  void inverse_kinematics(float x, float y, float z);
  void restrict2Workspace(float thetaW, float radiusW, float zW,bool craneMecanismFlag);
  /*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
  void setTargetJoints();
  void reachPosition(float deltaTime);
  void moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution);

  void updateMotors(unsigned char index);

  void jointExtremePosition(unsigned char index,unsigned char value);
  // Auxiliary testing methods
  void adjustMotorGains();
  
  void printMotorGains();
  void displayEncodersFrequency();
  
  void initializeVars();
};


#endif