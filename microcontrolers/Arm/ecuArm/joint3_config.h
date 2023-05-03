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

/*PID Control gains*/
#define KP_3                            1
#define KI_3                            1
#define KD_3                            1

/*PINS Description*/
#define POSITIVE_DIR_PIN_3                5
#define NEGATIVE_DIR_PIN_3                5
#define PWM_PIN_3                           0
#define LIMIT_SWITCH_H_PIN_3              9
#define LIMIT_SWITCH_L_PIN_3              9


#define ZERO_POS_3                      1.57