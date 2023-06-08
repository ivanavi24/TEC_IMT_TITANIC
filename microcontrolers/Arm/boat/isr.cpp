#include "isr.h"


#define LIMIT_SWITCH_HIGH_VALUE         1
#define LIMIT_SWITCH_LOW_VALUE          0



void IRAM_ATTR ISR__TIME_POSIITON()
{
    
}


/*GPIO Interrupt to read encoder*/
void IRAM_ATTR ISR__ENCODER_JOINT1(){


}
/*Limit Switch Interruption for Joint1*/
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT1(){

}
/*Limit Switch Interruption for Joint1*/
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT1(){

}

/*GPIO Interrupt to read encoder*/
void IRAM_ATTR ISR__ENCODER_JOINT2(){

}
/*Limit Switch Interruption for Joint2*/
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT2(){

}
/*Limit Switch Interruption for Joint2*/
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT2(){

}

void IRAM_ATTR ISR__LOW_VELOCITY_JOINT2(){

}

/*Ultrasonic Sensors Interruptions*/
void IRAM_ATTR ISR__TRIGG_ULTRASONIC()
{
    soundSystem.triggerSignal();
}
void IRAM_ATTR ISR__ECHO_ULTRASONIC_LEFT()
{
    soundSystem.updateSensorDistance(ULTR_SENSOR_LEFT);
}
void IRAM_ATTR ISR__ECHO_ULTRASONIC_CENTER()
{
    soundSystem.updateSensorDistance(ULTR_SENSOR_CENTER);
}
void IRAM_ATTR ISR__ECHO_ULTRASONIC_RIGHT()
{
    soundSystem.updateSensorDistance(ULTR_SENSOR_RIGHT);
}




/*Limit Switch Interruption for Joint3*/
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT3(){

}
/*Limit Switch Interruption for Joint3*/
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT3(){

}
