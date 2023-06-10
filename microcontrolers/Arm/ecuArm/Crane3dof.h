#include "motorsWencoder.h"
#include "motorStep.h"
#include "stateMachine.h"



extern stateMachine sequenceMachine;


#define CAMERA_UPDATE_TIME      20
#define CAMERA_PIN              13
#define CAMERA_PWM_CHANNEL      5
#define CAMERA_PWM_FREQUENCY    50
#define CAMERA_PWM_RESOLUTION   11
#define ZERO_PWM_DUTY_CYCLE     0.021
#define PI_PWM_DUTY_CYCLE       0.1250
#define PI                      3.1415926535
#define ZERO                    0
#define PI_DEGREES              180

#define MAIN_ROUTINE_TIMER        0

#define MAIN_ROUTINE_CONTROL_TIME 20000

//544 us 0
//926 us 90 delta
#ifndef CAMERA_SERVO_STATES
#define CAMERA_SERVO_STATES
enum CameraServoStates 
{
  not_scanning,
  scaning_towards,
  scaning_in_target_point,
  scaning_finished
};
#endif

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
    
    unsigned char camera_servo_pan_pos;
    unsigned char camera_servo_tilt_pos;

    hw_timer_t *My_timer = NULL;
    bool timerAttached =false;
    
    int cameraSweepCounter=0;
    int cameraChangeStep = 1;


    
    
    


  public:

  /*Global so it can be ser to zero from an external event*/
  int current_state_index_camera= 0; 
  CameraServoStates cameraScanningState;

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
  void reachPosition();
  void moveMotors(float pwm[], float minValue,float maxValue, int pwm_resolution);
  /*ISR intermedidate call to update correspondingMotion*/
  void updateMotors(unsigned char index);


  void moveCamera(float angle);
  

  /*Individual Motor movements*/
  void setTargetRPM(unsigned char index);
  void setTargetAngle(unsigned char index);
  void setTargetDisplacement(unsigned char index);
  void moveMotor(unsigned char index,float deltaTime);
  void moveMotorVel(unsigned char index, float deltaTime);
  void stopMotorMovement(unsigned char index);
  void stopAllMotors();
  
  /*Limit Switches ISR Handler*/
  void motorsLimitSwitchesHandler(unsigned char index, unsigned char value);
  
  
  void setZeroVelocityMotors(unsigned char index); 
  void initializeVars();
  void compareReferenceandCurrent(unsigned char index);
  unsigned char getCameraServoPanPos();
  unsigned char getCameraServoTiltPos();
  int getCraneThetaPos();

  void craneMovement();


  void resetcameraSweepCounter();

  void scanSorroundings();
};


#endif