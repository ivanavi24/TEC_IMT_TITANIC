#include "Crane3dof.h"
extern Crane3dof titanicCrane;
#define MOTOR1          1
#define MOTOR2          2
#define MOTOR3          3

#ifndef ISR_H
#define ISR_H
/*ISR calls to update motorsWencoder-> currenJoint value */
void IRAM_ATTR ISR__ENCODER_JOINT1();
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT1();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT1();


void IRAM_ATTR ISR__ENCODER_JOINT2();
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT2();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT2();


void IRAM_ATTR ISR__TIME_JOINT3();
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT3();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT3();
#endif