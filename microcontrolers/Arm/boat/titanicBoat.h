#include "motorsWencoder.h"
#include "ultrasonicSensor.h"
#include "stateMachine.h"



extern stateMachine sequenceMachine;

#define ZERO_PWM_DUTY_CYCLE     0.021
#define PI_PWM_DUTY_CYCLE       0.1250
#define PI                      3.1415926535
#define ZERO                    0
#define PI_DEGREES              180

#define MAIN_ROUTINE_TIMER        0

#define MAIN_ROUTINE_CONTROL_TIME 20000

//544 us 0
//926 us 90 delta

#ifndef BOAT_H
#define BOAT_H
class boat{
  private:

    float origin2water;
    DCmotor_Encoder right_motor;
    DCmotor_Encoder left_motor;

    float x_desired, y_desired, z_desired; /*Set thorugh i2c communication intereruption*/
    

   /**/

    hw_timer_t *My_timer = NULL;
    bool timerAttached =false;


  public:
  boat ();
    /*Assumes corrdinate frame located in the center of the robotic arm*/
  
  /*Getters methods*/
  DCmotor_Encoder get_right_motor();
  DCmotor_Encoder get_left_motor();


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
  void reachPosition();
  void moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution);
  /*ISR intermedidate call to update correspondingMotion*/
  void updateMotors(unsigned char index);



  

  /*Individual Motor movements*/
  void setTargetRPM(unsigned char index);
  void setTargetAngle(unsigned char index);
  void moveMotor(unsigned char index,float deltaTime);
  void moveMotorVel(unsigned char index, float deltaTime);
  void stopMotorMovement(unsigned char index);
  void stopAllMotors();
  
  /*Limit Switches ISR Handler*/
  void motorsLimitSwitchesHandler(unsigned char index, unsigned char value);
  
  
  void setZeroVelocityMotors(unsigned char index); 
  void initializeVars();
  void compareReferenceandCurrent(unsigned char index);
 

  void craneMovement();

  void s();


};


#endif