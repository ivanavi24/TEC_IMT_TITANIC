
#include "joints_config.h"
#include "Arduino.h"
#define STEP_MOTOR_CHANNELS             4
#ifndef MOTOR_STEP_H
#define MOTOR_STEP_H


class Step_motor{
  private: 


    PIN phase1;
    PIN phase2;
    PIN phase3;
    PIN phase4;
    PIN limit_switch_high;
    PIN limit_switch_low;

    PIN step_sequence[STEP_MOTOR_CHANNELS] ;
    unsigned char current_pin;
    unsigned int min_micros_between_step;
    unsigned int max_micros_between_step;


    /*Absolute HW limites defined by limit switches in terms of encoder pulses*/
    float joint_low_limit_hw;
    float joint_high_limit_hw;

    /*SW limits to restric movement within specified range*/
    float joint_low_limit_sw;
    float joint_high_limit_sw;

    /*Encoder resolution in pulses per revolution*/
    unsigned int steps_per_revolution;

    float joint_velocity_desired_RPM;
    
    
    float joint_error_i;    
    hw_timer_t *My_timer = NULL;
    bool timerAttached =false;
    int joint_desired;
    int joint_current;
    bool positiveMovementFlag=false;
    bool negativeMovementFlag=false;
    bool reachFlag =false;
    
  public:
    
    Step_motor(MotorStepParams motorParams);
    /*Getters*/

    void resetCurrentJoint();
    float getMotorRPM();
    int getJointCurrentVal();
    int getDesiredJointVal();
    void setJointDesired( float desired_revolutions);
    void setVelocityDesiredRPM( float desired_velocity);

    void initilizePINS();

    void move2position();
    void  moveDirection();
    void moveWVelocity(float deltaTime);
    void moveMotor();

    /*Refer to documentation https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf */
    void positiveMovement();
    void negativeMovement();
    void stopMovement();
    
    int satureControl(float control_action);
    
    void updateCurrentJoint(int value);
    
    void setJointDesiredFromAngle( float desired_angle);
    void setLimitSwitchReferencePoint(unsigned char value);

    float getMotorFrequency();
    
};

#endif

