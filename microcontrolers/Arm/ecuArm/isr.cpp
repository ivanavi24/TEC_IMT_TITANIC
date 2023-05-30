#include "isr.h"

#define LIMIT_SWITCH_HIGH_VALUE         1
#define LIMIT_SWITCH_LOW_VALUE          0
/*GPIO Interrupt to read encoder*/
void IRAM_ATTR ISR__ENCODER_JOINT1(){
    titanicCrane.updateMotors(MOTOR1);

}
/*Limit Switch Interruption for Joint1*/
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT1(){
    titanicCrane.motorsLimitSwitchesHandler(MOTOR1,LIMIT_SWITCH_HIGH_VALUE);
}
/*Limit Switch Interruption for Joint1*/
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT1(){
    titanicCrane.motorsLimitSwitchesHandler(MOTOR1,LIMIT_SWITCH_LOW_VALUE);
}

/*GPIO Interrupt to read encoder*/
void IRAM_ATTR ISR__ENCODER_JOINT2(){
    titanicCrane.updateMotors(MOTOR2);
}
/*Limit Switch Interruption for Joint2*/
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT2(){
    titanicCrane.motorsLimitSwitchesHandler(MOTOR2,LIMIT_SWITCH_HIGH_VALUE);
}
/*Limit Switch Interruption for Joint2*/
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT2(){
    titanicCrane.motorsLimitSwitchesHandler(MOTOR2,LIMIT_SWITCH_LOW_VALUE);
}

void IRAM_ATTR ISR__LOW_VELOCITY_JOINT2(){
    titanicCrane.setZeroVelocityMotors(MOTOR2);
}


/*Time interrupt ISR to move STEP MOTOR*/
void IRAM_ATTR ISR__TIME_JOINT3(){
    titanicCrane.updateMotors(MOTOR3);
}
/*Limit Switch Interruption for Joint3*/
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT3(){
    titanicCrane.motorsLimitSwitchesHandler(MOTOR3,LIMIT_SWITCH_HIGH_VALUE);
}
/*Limit Switch Interruption for Joint3*/
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT3(){
    titanicCrane.motorsLimitSwitchesHandler(MOTOR3,LIMIT_SWITCH_LOW_VALUE);
}
