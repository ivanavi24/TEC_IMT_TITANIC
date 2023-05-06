#include "joints_config.h"

#ifndef MOTORSWENCODER_H
#define MOTORSWENCODER_H
struct MotorEncoderParams{

    /*Encoder*/
    PIN encoderA;
    PIN encoderB;

    /*PWM*/
    PIN positive_dir_pin;
    PIN negative_dir_pin;
    PIN pwm_pin;
    
    /*Values for communication unwrappping*/
    unsigned int min_actuator_pwm_signal;
    unsigned int max_actuator_pwm_signal;
    
    float kp;
    float kd;
    float ki;

    /*Absolute HW limites defined by limit switches in terms of encoder pulses*/
    float joint_low_limit_hw;
    float joint_high_limit_hw;
    
    /*SW limits to restric movement within specified range*/
    float joint_low_limit_sw;
    float joint_high_limit_sw;
};
class DCmotor_Encoder{
  private: 
    int joint_desired;
    int joint_current;
    
    
    unsigned int min_actuator_signal;
    unsigned int max_actuator_signal;
    
    float kp, kd, ki;
    float joint_error_i;

    /*Absolute HW limites defined by limit switches in terms of encoder pulses*/
    float joint_low_limit_hw;
    float joint_high_limit_hw;
    
    /*SW limits to restric movement within specified range*/
    float joint_low_limit_sw;
    float joint_high_limit_sw;

    PIN encoderA;
    PIN encoderB;
    PIN positive_dir_pin;
    PIN negative_dir_pin;
    PIN pwm_pin;

    
  public:
    DCmotor_Encoder(MotorEncoderParams motorParams);
    void move2position(float deltaTime);
    void moveMotor(int duty_cycle);
    void moveDirection();
    /*Refer to documentation https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf */
    void positiveMovement();
    void negativeMovement();
    void stopMovement();
    unsigned int satureControl(float control_action);
    void setJointDesired( int desired_pulses);
    void updateCurrentJoint();
    void initializePWM(unsigned char ledchannel, unsigned int freq, unsigned char resolution);
    void displayGainValues();
    
};

#endif