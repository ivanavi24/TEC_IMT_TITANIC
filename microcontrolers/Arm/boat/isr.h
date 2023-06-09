#include "Arduino.h"
#include "ultrasonicSensor.h"
#define MOTOR1          1
#define MOTOR2          2
#define MOTOR3          3

#define ULTR_SENSOR_LEFT    1
#define ULTR_SENSOR_CENTER  2
#define ULTR_SENSOR_RIGHT   3
extern ultrasonicSensor soundSystem;
#ifndef ISR_H
#define ISR_H




/*Ultrasonic sensor time isr*/
void IRAM_ATTR ISR__TRIGG_ULTRASONIC();
void IRAM_ATTR ISR__ECHO_ULTRASONIC_LEFT();
void IRAM_ATTR ISR__ECHO_ULTRASONIC_CENTER();
void IRAM_ATTR ISR__ECHO_ULTRASONIC_RIGHT();



void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT1();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT1();
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT2();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT2();
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT3();
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT3();

#endif