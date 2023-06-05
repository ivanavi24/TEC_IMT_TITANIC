#include "motorsWencoder.h"
#include "motorStep.h"
#include "stateMachine.h"



extern stateMachine sequenceMachine;


#ifndef CRANE3DOF_H
#define CRANE3DOF_H
class Crane3dof{
  private:

    float origin2water;
    DCmotor_Encoder first_motor;
    DCmotor_Encoder second_motor;
    Step_motor third_motor;


    float x_desired, y_desired, z_desired; /*Set thorugh i2c communication intereruption*/
    

   /**/
    float theta,radius,z_height;
    
    int camera_servo_pan_pos;
    int camera_servo_tilt_pos;

  public:
  Crane3dof ();
    /*Assumes corrdinate frame located in the center of the robotic arm*/
  
  /*Getters methods*/
  DCmotor_Encoder get_first_motor();
  DCmotor_Encoder get_second_motor();
  Step_motor get_third_motor();

  float getXdesired();
  float getYdesired();
  float getZdesired();
  


  void adjustMotorGains(unsigned char index,bool velocityGains);
  void printMotorGains(unsigned char index);
  void displayEncodersFrequency(unsigned char index);


  /*Inverse kinematic calculate */
  void set_target_position(float x, float y, float z);
  bool inverse_kinematics(float x, float y, float z);
  bool restrict2Workspace(float thetaW, float radiusW, float zW);


  /*Convert (X,Y,Z) coordinates into joint pulses and update desired values for each motor*/
  void setTargetJoints();
  void reachPosition(float deltaTime);
  void moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution);
  /*ISR intermedidate call to update correspondingMotion*/
  void updateMotors(unsigned char index);

  // Auxiliary testing methods
  

  /*Individual Motor movements*/
  void setTargetRPM(unsigned char index);
  void setTargetAngle(unsigned char index);
  void moveMotor(unsigned char index,float deltaTime);
  void moveMotorVel(unsigned char index, float deltaTime);
  void moveAllMotors(float deltaTime);
  void stopMotorMovement(unsigned char index);
  void stopAllMotors();
  
  /*Limit Switches ISR Handler*/
  void motorsLimitSwitchesHandler(unsigned char index, unsigned char value);
  
  
  void setZeroVelocityMotors(unsigned char index); 
  void initializeVars();
  void compareReferenceandCurrent(unsigned char index);
  int getCameraServoPanPos();
  int getCameraServoTiltPos();
  int getCraneThetaPos();
};


#endif