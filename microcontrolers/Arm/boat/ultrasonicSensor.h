#include "joints_config.h"
#include "Arduino.h"



#define MICROSECONDS_DELAY              10
#define MAX_SECONDS_INPUT               20
#define MIN_SECONDS_INPUT                10

#define PRESCALER_2_MICROSECONDS         80


#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

class ultrasonicSensor
{
    private:
        PIN echo_pin_left;
        PIN echo_pin_right;
        PIN echo_pin_center;

        PIN trigger_pin_left;
        PIN trigger_pin_center;
        PIN trigger_pin_right;

        long t_left=0;
        long t_last_left=0;
        

        long t_center=0;
        long t_last_center=0;
        

        long t_right=0;
        long t_last_right=0;
        
        
        hw_timer_t *trigSensorTimer = NULL;

        bool trig_high_value = false;

    public:
    
    long d_center=0;
    long d_left=0;
    long d_right=0;
    int distance =0;
    ultrasonicSensor();
    
    void initialize();

    void triggerSignal();

    void updateDistances();
    void updateSensorDistance(unsigned char sensor_selector);



};


#endif
