#ifndef MOTORSWENCODER_H
#define MOTORSWENCODER_H
class DCmotor_Encoder{
  private: 
    int joint_desired;
    int joint_current;
    unsigned char positive_dir_pin;
    unsigned char negative_dir_pin;
    unsigned char pwm_pin;
    
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

    
  public:
    DCmotor_Encoder(float kpV, float kiV, float kdV, unsigned char positive_dir_pinV,unsigned char negative_dir_pinV,unsigned char pwm_pinV,float joint_low_limit_hwV,
                   float  joint_high_limit_hwV, float joint_low_limit_swV, float joint_high_limit_swV);
    void move2position(float deltaTime);
    void moveMotor(int duty_cycle);
    void moveDirection();
    /*Refer to documentation https://www.sparkfun.com/datasheets/Robotics/L298_H_Bridge.pdf */
    void positiveMovement();
    void negativeMovement();
    void stopMovement();
    unsigned int satureControl(float control_action);
    void setJointDesired( int desired_pulses);
};

#endif