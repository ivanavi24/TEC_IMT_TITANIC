
#include "ultrasonicSensor.h"
#include "ultrasonicSensor_config.h"
#include "isr.h"


#define ULTR_SENSOR_LEFT    1
#define ULTR_SENSOR_CENTER  2
#define ULTR_SENSOR_RIGHT   3
#define   ULTR_TIMER        2
extern int counterBaby; 
ultrasonicSensor::ultrasonicSensor()
{
    trigSensorTimer = timerBegin(ULTR_TIMER, PRESCALER_2_MICROSECONDS, true);

    
    trigger_pin_left = ULTRA_TRIG_LEFT;
    echo_pin_left = ULTRA_ECHO_LEFT;
    
    trigger_pin_center = ULTRA_TRIG_CENTER;
    echo_pin_center = ULTRA_ECHO_CENTER;

    trigger_pin_right = ULTRA_TRIG_RIGHT;
    echo_pin_right = ULTRA_ECHO_RIGHT;


    


    
    

}
void ultrasonicSensor::triggerSignal()
{
    counterBaby++;

    if (trig_high_value){
      digitalWrite(trigger_pin_left,HIGH);
      digitalWrite(trigger_pin_right,HIGH);
      digitalWrite(trigger_pin_center,HIGH);
      timerDetachInterrupt(trigSensorTimer);
      timerAttachInterrupt(trigSensorTimer, &ISR__TRIGG_ULTRASONIC, true);
      trig_high_value= false;
    }
    else {
      digitalWrite(trigger_pin_left,LOW);
      digitalWrite(trigger_pin_right,LOW);
      digitalWrite(trigger_pin_center,LOW);
      trig_high_value= true;
      timerDetachInterrupt(trigSensorTimer);
    
    }
}

void ultrasonicSensor::updateDistances()
{
  timerAttachInterrupt(trigSensorTimer, &ISR__TRIGG_ULTRASONIC, true);
}


void ultrasonicSensor::updateSensorDistance(unsigned char sensor_selector)
{
  switch (sensor_selector)
    {
    case ULTR_SENSOR_LEFT:
        if (digitalRead(echo_pin_left)){
          t_last_left = micros();
        }
        else {
          t_left = micros()-t_last_left;
          if(t_left<12000)
            d_left = t_left/59;
          
        }

        break;
    case ULTR_SENSOR_CENTER:
        if (digitalRead(echo_pin_center)){
          t_last_center = micros();
        }
        else {
          t_center = micros()-t_last_center;
          if(t_center<12000)
            d_center = t_center/59;
          
        }
        break;
    case ULTR_SENSOR_RIGHT:
        if (digitalRead(echo_pin_right)){
          t_last_right = micros();
        }
        else {
          t_right = micros()-t_last_right;
          if(t_right<12000)
            d_right = t_right/59;
        }

        break;       
    default:
        break;
    }
}

void ultrasonicSensor::initialize()
{
    pinMode(trigger_pin_left,OUTPUT);
    pinMode(trigger_pin_center,OUTPUT);
    pinMode(trigger_pin_right,OUTPUT);
    pinMode(echo_pin_right,INPUT);
    pinMode(echo_pin_left,INPUT);
    pinMode(echo_pin_center,INPUT);

   attachInterrupt(echo_pin_left, ISR__ECHO_ULTRASONIC_LEFT, CHANGE); 
   attachInterrupt(echo_pin_right, ISR__ECHO_ULTRASONIC_RIGHT, CHANGE);
  attachInterrupt(echo_pin_center, ISR__ECHO_ULTRASONIC_CENTER, CHANGE); 

    timerAlarmWrite(trigSensorTimer, 10, true);
   
    timerAlarmEnable(trigSensorTimer);
}



 