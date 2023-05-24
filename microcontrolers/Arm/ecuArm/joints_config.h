
#define ZERO_VAL_INITIALIZER  0

#define SECONDS_IN_MINUTES   60

#ifndef MOTOR_ENCODER_STRUCT
#define MOTOR_ENCODER_STRUCT

typedef unsigned char PIN;
struct MotorDCEncoderParams{

    /*Encoder*/
    PIN encoderA;
    PIN encoderB;
    /*PWM*/
    PIN positive_dir_pin;
    PIN negative_dir_pin;
    PIN pwm_pin;

    PIN limit_switch_high;
    PIN limit_switch_low;
    
    /*Values for communication unwrappping*/
    unsigned int min_actuator_pwm_signal;
    unsigned int max_actuator_pwm_signal;
    unsigned char pwm_channel;
    unsigned int pwm_frquency;
    unsigned char pwm_resolution;
    
    float kp;
    float kd;
    float ki;
    float kp_v;
    float kd_v;
    float ki_v;

    /*Absolute HW limites defined by limit switches in terms of encoder pulses*/
    float joint_low_limit_hw;
    float joint_high_limit_hw;
    
    /*SW limits to restric movement within specified range*/
    float joint_low_limit_sw;
    float joint_high_limit_sw;

    /*Encoder resolutions in pulses per revolution*/
    unsigned int encoder_resolution;

    unsigned int average_pulses;
};
struct MotorStepParams{

    /*INPUTS */
    PIN IN1_PIN;
    PIN IN2_PIN;
    PIN IN3_PIN;
    PIN IN4_PIN;
    /*Limit switches*/
    PIN limit_switch_high;
    PIN limit_switch_low;
    
    /*Values for communication unwrappping*/
    unsigned int min_micros_between_step;
    unsigned int max_micros_between_step;

    /*Absolute HW limites defined by limit switches in terms of encoder pulses*/
    float joint_low_limit_hw;
    float joint_high_limit_hw;
    
    /*SW limits to restric movement within specified range*/
    float joint_low_limit_sw;
    float joint_high_limit_sw;

    /*Encoder resolutions in pulses per revolution*/
    unsigned int steps_per_revolution;
};


#endif