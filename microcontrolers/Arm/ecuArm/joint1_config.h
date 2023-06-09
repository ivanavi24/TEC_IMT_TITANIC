
#include "joints_config.h"

#ifndef JOINT1_PARAMS_
#define JOINT1_PARAMS_
#define JOINT1_ZERO_POSITION            60.5

/*Encoder resoluition in pulses per meter [pulses/m]*/
#define JOINT1_ENCODER_RESOLUTION       244

/*Joint movement limit in terms of pulses 
(Low limit is 0, since start rutine sets zero with limite switch)
Arm movement preferable be within the HW limits*/ 
#define JOINT1_LOW_LIMIT_HW                0   
#define JOINT1_HIGH_LIMIT_HW               90  

/**SW limits to restrict movement wtithin a given range*/
#define JOINT1_LOW_LIMIT_SW                0   
#define JOINT1_HIGH_LIMIT_SW               90  

#define MIN_PWM_SIGNAL              0
#define MAX_PWM_SIGNAL              1023
/*PID Control gains*/
#define KP_1                            1
#define KI_1                            0
#define KD_1                            0
#define KP_V_1                          1
#define KD_V_1                          1
#define KI_V_1                          1


/*PINS Description*/
#define POSITIVE_DIR_PIN_1                  5
#define NEGATIVE_DIR_PIN_1                  19
#define PWM_PIN_1                           18
#define ENCODER_A_1                         34
#define ENCODER_B_1                         35
#define LIMIT_SWITCH_H_PIN_1                9
#define LIMIT_SWITCH_L_PIN_1                9
/*
#define POSITIVE_DIR_PIN_1                  4
#define NEGATIVE_DIR_PIN_1                  17
#define PWM_PIN_1                           16
#define ENCODER_A_1                         34
#define ENCODER_B_1                         35
#define LIMIT_SWITCH_H_PIN_1                9
#define LIMIT_SWITCH_L_PIN_1                9


*/
/*PWM configuration*/
#define PWM_CHANNEL_1                       0
#define PWM_FREQUENCY_1                   5000
#define PWM_RESOLUTION_1                    10

#define JOINT1_DEFECT_ENCODER               false

#define AVG_VEL_NUM_PULSES_1                50


#define ZERO_POS_1                      1.57
#define REVOLUTIONS_PER_METER_1              1  //should be one in the first joint since movement is revolute

struct MotorDCEncoderParams joint1 = 
{
    ENCODER_A_1,
    ENCODER_B_1,
    POSITIVE_DIR_PIN_1,
    NEGATIVE_DIR_PIN_1,
    PWM_PIN_1,
    LIMIT_SWITCH_H_PIN_1,
    LIMIT_SWITCH_L_PIN_1,
    MIN_PWM_SIGNAL,
    MAX_PWM_SIGNAL,
    PWM_CHANNEL_1,
    PWM_FREQUENCY_1,
    PWM_RESOLUTION_1,
    KP_1,
    KD_1,
    KI_1,
    KP_V_1,
    KD_V_1,
    KI_V_1,
    JOINT1_LOW_LIMIT_HW,        
    JOINT1_HIGH_LIMIT_HW, 
    JOINT1_LOW_LIMIT_SW,                  
    JOINT1_HIGH_LIMIT_SW ,
    JOINT1_ENCODER_RESOLUTION,
    AVG_VEL_NUM_PULSES_1,
    JOINT1_DEFECT_ENCODER,
    REVOLUTIONS_PER_METER_1
    
    
     
};

#endif