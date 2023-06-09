#include "isr.h"

#define LIMIT_SWITCH_HIGH_VALUE         1
#define LIMIT_SWITCH_LOW_VALUE          0

void IRAM_ATTR ISR__TIME_POSIITON()
{
    titanicCrane.craneMovement();
    
}


/*GPIO Interrupt to read encoder*/
void IRAM_ATTR ISR__ENCODER_JOINT1(){
    titanicCrane.updateMotors(MOTOR1);

}


/*GPIO Interrupt to read encoder*/
void IRAM_ATTR ISR__ENCODER_JOINT2(){
    titanicCrane.updateMotors(MOTOR2);
}


void IRAM_ATTR ISR__LOW_VELOCITY_JOINT2(){
    titanicCrane.setZeroVelocityMotors(MOTOR2);
}


/*Time interrupt ISR to move STEP MOTOR*/
void IRAM_ATTR ISR__TIME_JOINT3(){
    titanicCrane.updateMotors(MOTOR3);
}

