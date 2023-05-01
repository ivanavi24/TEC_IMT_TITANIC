#define JOINT1_ZERO_POSITION            60.5

/*Encoder resoluition in pulses per revolution [pulses/revolution]*/
#define JOINT1_ENCODER_RESOLUTION       50

/*Joint movement limit in terms of pulses 
(Low limit is 0, since start rutine sets zero with limite switch)
Arm movement preferable be within the HW limits*/ 
#define JOINT1_LOW_LIMIT_HW                0   
#define JOINT1_HIGH_LIMIT_HW               90  /*First Joint movement limit in terms of pulses*/

/**SW limits to restrict movement wtithin a given range*/
#define JOINT1_LOW_LIMIT_SW                 0   /*First Joint movement limit in terms of pulses (Low limit is 0, since start rutine sets zero with limite switch)*/ 
#define JOINT1_HIGH_LIMIT_SW               90  /*First Joint movement limit in terms of pulses*/

/*PID Control gains*/
#define KP_1                            1
#define KI_1                            1
#define KD_1                            1

/*PINS Description*/
#define POSITIVE_DIR_PIN_1              5
#define NEGATIVE_DIR_PIN_1              5
#define LIMIT_SWITCH_H_PIN_1              9
#define LIMIT_SWITCH_L_PIN_1              9


#define ZERO_POS_1                      1.57





