#include "Arduino.h"
extern Crane3dof titanicCrane;
#define MOTOR1          1
#define MOTOR2          2
#define MOTOR3          3

/*ISR calls to update motorsWencoder-> currenJoint value */
void ISR__ENCODER_JOINT1(){
    titanicCrane.updateMotors(MOTOR1);
}
void ISR__ENCODER_JOINT2(){
    titanicCrane.updateMotors(MOTOR2);
}
void ISR__ENCODER_JOINT3(){
    titanicCrane.updateMotors(MOTOR3);
}