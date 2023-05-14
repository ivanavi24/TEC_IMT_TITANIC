#include "joints_config.h"

#ifndef JOINT3_PARAMS_
#define JOINT3_PARAMS_

#define JOINT3_ZERO_POSITION            60.5

/*Encoder resoluition in pulses per meter [pulses/m]*/
#define JOINT3_ENCODER_RESOLUTION       50

/*Joint movement limit in terms of pulses 
(Low limit is 0, since start rutine sets zero with limite switch)
Arm movement preferable be within the HW limits*/ 
#define JOINT3_LOW_LIMIT_HW                0   
#define JOINT3_HIGH_LIMIT_HW               90  

/**SW limits to restrict movement wtithin a given range*/
#define JOINT3_LOW_LIMIT_SW                0   
#define JOINT3_HIGH_LIMIT_SW               90  

#define MIN_PWM_SIGNAL              0
#define MAX_PWM_SIGNAL              0
/*PID Control gains*/
#define KP_3                            2
#define KI_3                            7
#define KD_3                            4
#define KP_V_3                          1
#define KD_V_3                          1
#define KI_V_3                          1

/*PINS Description*/
#define POSITIVE_DIR_PIN_3                5
#define NEGATIVE_DIR_PIN_3                5
#define PWM_PIN_3           0
#define ENCODER_A_3                     24
#define ENCODER_B_3                     25
#define LIMIT_SWITCH_H_PIN_3              9
#define LIMIT_SWITCH_L_PIN_3              9

/*PWM configuration*/
#define PWM_CHANNEL_3                       0
#define PWM_FREQUENCY_3                   5000
#define PWM_RESOLUTION_3                    16


#define AVG_VEL_NUM_PULSES_3                10

#define ZERO_POS_3                      1.57
#define REVOLUTIONS_PER_METER_3              1  //should be one in the first joint since movement is revolute

struct MotorEncoderParams joint3 = 
{
    ENCODER_A_3,
    ENCODER_B_3,
    POSITIVE_DIR_PIN_3,
    NEGATIVE_DIR_PIN_3,
    PWM_PIN_3,
    LIMIT_SWITCH_H_PIN_3,
    LIMIT_SWITCH_L_PIN_3,
    MIN_PWM_SIGNAL,
    MAX_PWM_SIGNAL,
    PWM_CHANNEL_3,
    PWM_FREQUENCY_3,
    PWM_RESOLUTION_3,
    KP_3,
    KD_3,
    KI_3,
    KP_V_3,
    KD_V_3,
    KI_V_3,
    JOINT3_LOW_LIMIT_HW,        
    JOINT3_HIGH_LIMIT_HW, 
    JOINT3_LOW_LIMIT_SW,                  
    JOINT3_HIGH_LIMIT_SW,
    JOINT3_ENCODER_RESOLUTION,
    AVG_VEL_NUM_PULSES_3  
};

#endif