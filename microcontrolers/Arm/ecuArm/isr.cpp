
#include "isr.h"


void IRAM_ATTR ISR__ENCODER_JOINT1(){
    titanicCrane.updateMotors(MOTOR1);

}
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT1(){
    titanicCrane.updateMotors(MOTOR1);
}
void ISR__LIMIT_SWITCH_L_JOINT1(){
    titanicCrane.updateMotors(MOTOR1);
}


void ISR__ENCODER_JOINT2(){
    titanicCrane.updateMotors(MOTOR2);
}
void ISR__LIMIT_SWITCH_H_JOINT2(){
    titanicCrane.updateMotors(MOTOR1);
}
void ISR__LIMIT_SWITCH_L_JOINT2(){
    titanicCrane.updateMotors(MOTOR1);
}


void ISR__ENCODER_JOINT3(){
    titanicCrane.updateMotors(MOTOR3);
}
void ISR__LIMIT_SWITCH_H_JOINT3(){
    titanicCrane.updateMotors(MOTOR1);
}
void ISR__LIMIT_SWITCH_L_JOINT3(){
    titanicCrane.updateMotors(MOTOR1);
}
