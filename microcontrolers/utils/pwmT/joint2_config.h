#define JOINT2_ZERO_POSITION            60.5

/*Encoder resoluition in pulses per meter [pulses/m]*/
#define JOINT2_ENCODER_RESOLUTION       50

/*Joint movement limit in terms of pulses 
(Low limit is 0, since start rutine sets zero with limite switch)
Arm movement preferable be within the HW limits*/ 
#define JOINT2_LOW_LIMIT_HW                0  
#define JOINT2_HIGH_LIMIT_HW               90  

/**SW limits to restrict movement wtithin a given range*/
#define JOINT2_LOW_LIMIT_SW                0   
#define JOINT2_HIGH_LIMIT_SW               90 

/*PID Control gains*/
#define KP_2                            1
#define KI_2                            1
#define KD_2                            1

/*PINS Description*/
#define POSITIVE_DIR_PIN_2                5
#define NEGATIVE_DIR_PIN_2                5
#define PWM_PIN_2                         19
#define LIMIT_SWITCH_H_PIN_2              9
#define LIMIT_SWITCH_L_PIN_2              9

/*PWM configuration*/
#define PWM_CHANNEL_2                       2
#define PWM_FREQUENCY_2                   2000
#define PWM_RESOLUTION_2                    8

#define ZERO_POS_2                      1.57