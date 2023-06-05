#include "joints_config.h"

#ifndef JOINT3_PARAMS_
#define JOINT3_PARAMS_

#define JOINT3_ZERO_POSITION            60.5

/*Encoder resoluition in STEPS per Revolution*/
#define JOINT3_ENCODER_RESOLUTION       200

/*Joint movement limit in terms of pulses 
(Low limit is 0, since start rutine sets zero with limite switch)
Arm movement preferable be within the HW limits*/ 
#define JOINT3_LOW_LIMIT_HW                0   
#define JOINT3_HIGH_LIMIT_HW               90  

/**SW limits to restrict movement wtithin a given range*/
#define JOINT3_LOW_LIMIT_SW                0   
#define JOINT3_HIGH_LIMIT_SW               90  

/*PINS Description*/
#define IN1_PIN                                 32
#define IN2_PIN                                 33
#define IN3_PIN                                 25
#define IN4_PIN                                 26
#define LIMIT_SWITCH_H_PIN_3              9
#define LIMIT_SWITCH_L_PIN_3              9

/*Define maximum movement velocity in terms of time between steps*/
#define MIN_MICROS_BETWEEN_STEPS                 870
#define MAX_MICROS_BETWEEN_STEPS                 2500
#define NOMRAL_MICROS_BEWTEEN_STEPS              2100 

#define ZERO_POS_3                      1.57
#define REVOLUTIONS_PER_METER_3              1  //should be one in the first joint since movement is revolute
#define INIT_POS_HALF_LINE                  0.2
struct MotorStepParams joint3{
    IN1_PIN,
    IN2_PIN,
    IN3_PIN,
    IN4_PIN,
    LIMIT_SWITCH_H_PIN_3,
    LIMIT_SWITCH_L_PIN_3,
    MIN_MICROS_BETWEEN_STEPS,
    MAX_MICROS_BETWEEN_STEPS,
    JOINT3_LOW_LIMIT_HW,
    JOINT3_HIGH_LIMIT_HW,
    JOINT3_LOW_LIMIT_SW,
    JOINT3_HIGH_LIMIT_HW,
    JOINT3_ENCODER_RESOLUTION,
    NOMRAL_MICROS_BEWTEEN_STEPS

};

#endif