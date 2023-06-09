#include "isr.h"
#include "titanicBoat.h"
#include "Arduino.h"

#define LIMIT_SWITCH_HIGH_VALUE         1
#define LIMIT_SWITCH_LOW_VALUE          0

extern boat rose;

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




void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT1()
{
    unsigned char index = 1;
    if (digitalRead(rose.LimitSwitch_j1_high))
    {
        rose.limitSwitches |= 1<< index;
    }

    else
    {
        rose.limitSwitches &= ~(1<< index);

    }
   

}
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT1()
{
    unsigned char index = 2;
    if (digitalRead(rose.LimitSwitch_j1_low))
    {
        rose.limitSwitches |= 1<< index;
    }

    else
    {
        rose.limitSwitches &= ~(1<< index);

    }
}
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT2()
{
    unsigned char index = 3;
    if (digitalRead(rose.LimitSwitch_j2_high))
    {
        rose.limitSwitches |= 1<< index;
    }

    else
    {
        rose.limitSwitches &= ~(1<< index);

    }

}
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT2(){

    unsigned char index = 4;
    if (digitalRead(rose.LimitSwitch_j2_low))
    {
        rose.limitSwitches |= 1<< index;
    }

    else
    {
        rose.limitSwitches &= ~(1<< index);

    }


}
void IRAM_ATTR ISR__LIMIT_SWITCH_H_JOINT3()
{
    unsigned char index = 5;
    if (digitalRead(rose.LimitSwitch_j2_low))
    {
        rose.limitSwitches |= 1<< index;
    }

    else
    {
        rose.limitSwitches &= ~(1<< index);

    }
}
void IRAM_ATTR ISR__LIMIT_SWITCH_L_JOINT3()
{
    unsigned char index = 6;
    if (digitalRead(rose.LimitSwitch_j3_low))
    {
        rose.limitSwitches |= 1<< index;
    }

    else
    {
        rose.limitSwitches &= ~(1<< index);

    }
}


