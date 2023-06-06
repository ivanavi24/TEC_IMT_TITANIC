#include "joints_config.h"

#ifndef JOINT2_PARAMS_
#define JOINT2_PARAMS_

#define JOINT2_ZERO_POSITION            60.5

/*Encoder resoluition in pulses per meter [pulses/m]*/
#define JOINT2_ENCODER_RESOLUTION       920*18

/*Joint movement limit in terms of pulses 
(Low limit is 0, since start rutine sets zero with limite switch)
Arm movement preferable be within the HW limits*/ 
#define JOINT2_LOW_LIMIT_HW                0   
#define JOINT2_HIGH_LIMIT_HW               90  

/**SW limits to restrict movement wtithin a given range*/
#define JOINT2_LOW_LIMIT_SW                0   
#define JOINT2_HIGH_LIMIT_SW               90  

#define MIN_PWM_SIGNAL              0
#define MAX_PWM_SIGNAL              4000
/*PID Control gains*/
#define KP_2                          0
#define KI_2                          9
#define KD_2                          1
#define KP_V_2                          1
#define KD_V_2                          1
#define KI_V_2                          1


/*PINS Description*/
#define POSITIVE_DIR_PIN_2              12
#define NEGATIVE_DIR_PIN_2              13
#define PWM_PIN_2         14
#define ENCODER_A_2                   34
#define ENCODER_B_2                   35
#define LIMIT_SWITCH_H_PIN_2            9
#define LIMIT_SWITCH_L_PIN_2            9

/*PWM configuration*/
#define PWM_CHANNEL_2                     0
#define PWM_FREQUENCY_2                 5000
#define PWM_RESOLUTION_2                  10

#define JOINT2_DEFECT_ENCODER               true

#define AVG_VEL_NUM_PULSES_2                2

#define ZERO_POS_2                    1.57
#define REVOLUTIONS_PER_METER_2              1  //should be one in the first joint since movement is revolute

struct MotorDCEncoderParams joint2 = 
{
    ENCODER_A_2,
    ENCODER_B_2,
    POSITIVE_DIR_PIN_2,
    NEGATIVE_DIR_PIN_2,
    PWM_PIN_2,
    LIMIT_SWITCH_H_PIN_2,
    LIMIT_SWITCH_L_PIN_2,
    MIN_PWM_SIGNAL,
    MAX_PWM_SIGNAL,
    PWM_CHANNEL_2,
    PWM_FREQUENCY_2,
    PWM_RESOLUTION_2,
    KP_2,
    KD_2,
    KI_2,
    KP_V_2,
    KD_V_2,
    KI_V_2,
    JOINT2_LOW_LIMIT_HW,        
    JOINT2_HIGH_LIMIT_HW, 
    JOINT2_LOW_LIMIT_SW,                  
    JOINT2_HIGH_LIMIT_SW ,
    JOINT2_ENCODER_RESOLUTION,
    AVG_VEL_NUM_PULSES_2,
    JOINT2_DEFECT_ENCODER
    
};



#endif