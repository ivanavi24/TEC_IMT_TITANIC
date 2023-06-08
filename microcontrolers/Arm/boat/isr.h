#include "Arduino.h"
#define MOTOR1          1
#define MOTOR2          2
#define MOTOR3          3

#define ULTR_SENSOR_LEFT    1
#define ULTR_SENSOR_CENTER  2
#define ULTR_SENSOR_RIGHT   3

#ifndef ISR_H
#define ISR_H


void IRAM_ATTR ISR__TIME_POSIITON();
/*ISR calls to update motorsWencoder-> currenJoint value */
void IRAM_ATTR ISR__ENCODER_JOINT1();
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT1();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT1();


/*Ultrasonic sensor time isr*/
void IRAM_ATTR ISR__TRIGG_ULTRASONIC();
void IRAM_ATTR ISR__ECHO_ULTRASONIC_LEFT();
void IRAM_ATTR ISR__ECHO_ULTRASONIC_CENTER();
void IRAM_ATTR ISR__ECHO_ULTRASONIC_RIGHT();



void IRAM_ATTR ISR__LOW_VELOCITY_JOINT2();


void IRAM_ATTR ISR__TIME_JOINT3();
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT3();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT3();
#endif