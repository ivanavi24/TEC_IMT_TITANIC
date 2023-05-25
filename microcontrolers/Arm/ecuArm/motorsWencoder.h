#include "joints_config.h"
#include "Arduino.h"
#ifndef MOTORSWENCODER_H
#define MOTORSWENCODER_H

class DCmotor_Encoder{
  private: 


    PIN encoderA;
    PIN encoderB;
    PIN positive_dir_pin;
    PIN negative_dir_pin;
    PIN pwm_pin;
    PIN limit_switch_high;
    PIN limit_switch_low;

    unsigned int min_actuator_signal;
    unsigned int max_actuator_signal;

    unsigned char pwm_channel;
    unsigned int pwm_frquency;
    unsigned char pwm_resolution;

    float kp, kd, ki;
    float kp_v, kd_v, ki_v;


    /*Absolute HW limites defined by limit switches in terms of encoder pulses*/
    float joint_low_limit_hw;
    float joint_high_limit_hw;

    /*SW limits to restric movement within specified range*/
    float joint_low_limit_sw;
    float joint_high_limit_sw;

    /*Encoder resolution in pulses per revolution*/
    unsigned int encoder_resolution;
    unsigned int average_pulses;

    int joint_desired;
    int joint_current;
    float joint_error_i;

    float joint_velocity_desired;
    float  joint_velocity_error_i;
    
    double avg_vel_pps_current;
    
    

    
    
  public:
    
    DCmotor_Encoder(MotorDCEncoderParams motorParams);
    /*Getters*/
    float getMotorRPM();
    float getMotorFrequency();
    PIN getEncoderA();
    int getJointCurrentVal();
    int getDesiredJointVal();

    void setPositionGains(float kpV, float kdV, float kiV);
    void setVelocityGains(float kpV, float kdV, float kiV);

    void move2position(float deltaTime);
    void moveWVelocity(float deltaTime);
    void moveMotor(int duty_cycle);
    void moveDirection();
    void moveDirectionVelocity();
    /*Refer to documentation https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf */
    void positiveMovement();
    void negativeMovement();
    void stopMovement();
    
    int satureControl(float control_action);
    void setJointDesired(float desired_revolutions);
    void setVelocityDesiredRPM( float desired_velocity);
    void updateCurrentJoint();
    
    void setJointDesiredFromAngle( float desired_angle);
    void setReferencePoint(unsigned char value);

    void displayGainValues();

    void initializePWM();
    void initilizeEncoders();
    
    
};

#endif