#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include "BluetoothSerial.h"
#define PWM_LEFT  13
#define DIR1_LEFT  27
//#define DIR2_LEFT   14

#define PWM_RIGHT       12
#define DIR1_RIGHT      14
//#define DIR2_RIGHT      4

// setting PWM properties
const int freq = 5000;
const int ledChannel_right = 0;
const int ledChannel_left = 2;
const int resolution = 8;

BluetoothSerial SerialBT;
void setup(){
  // configure LED PWM functionalitites
  
  pinMode(PWM_LEFT,OUTPUT);
  pinMode(DIR1_LEFT,OUTPUT);
  //pinMode(DIR2_LEFT,OUTPUT);
  pinMode(PWM_RIGHT,OUTPUT);
  pinMode(DIR1_RIGHT,OUTPUT);
  //pinMode(DIR2_RIGHT,OUTPUT);
  
  
  ledcSetup(ledChannel_right, freq, resolution);
  ledcSetup(ledChannel_left, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(PWM_LEFT, ledChannel_left);
  ledcAttachPin(PWM_RIGHT, ledChannel_right);
  
  //Serial.begin(115200);
  SerialBT.begin("Titanic"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  digitalWrite(DIR1_LEFT,LOW);
  //digitalWrite(DIR2_LEFT,HIGH);
  digitalWrite(DIR1_RIGHT,LOW);
  //digitalWrite(DIR2_RIGHT,HIGH);
}
 
void loop(){
  // increase the LED brightness
  
    // changing the LED brightness with PWM
    //ledcWrite(ledChannel, dutyCycle);
  while(SerialBT.available()<2){

  }

  if(SerialBT.available()==2){
    Serial.printf ("cantidad: %u\n",SerialBT.available());
    unsigned char pwm_right_motor=SerialBT.read();
    unsigned char pwm_left_motor=SerialBT.read();
    //Serial.printf("The pwm in right motor is %u \n",pwm_right_motor);
    //Serial.printf("The pwm in left motor is %u \n",pwm_left_motor);
    ledcWrite(ledChannel_right, pwm_right_motor);
    ledcWrite(ledChannel_left, pwm_left_motor);
    
  }
  else
 {
   while(SerialBT.available()){
    SerialBT.read();
   }
 }
  

    
    

 
  
}
