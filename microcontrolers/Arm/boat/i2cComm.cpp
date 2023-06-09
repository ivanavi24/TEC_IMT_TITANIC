#include "Wire.h"
#include "i2c_config.h"
#include "i2cComm.h"
#include "Arduino.h"
#include "stateMachine.h"


extern stateMachine sequenceMachine;
unsigned int valor=90;
void i2c_onRequest(){
  
  /*Write LS values and current estate of boat*/


  if ((sequenceMachine.getCurrentState()==sailing)|(sequenceMachine.getCurrentState()==exploring))
  {
     Wire.write(sailing);
  }
  else
  {
    Wire.write(recolection);
  }
 
  Wire.write(sequenceMachine.getCurrentState());

}
int readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
  return bComposed;
}
void i2c_onReceive(int len){

  int tempByte = Wire.read(); //read one extra due to format
  tempByte = Wire.read();
  switch (tempByte)
  {
  case MOVE_BOAT_COMMAND:
  {

    tempByte = Wire.read();
    if(tempByte == END_COMMAND){

      sequenceMachine.changeState(exploring);
      Serial.println("Exploring into new waters");
    } 
    break;
  }

    
  default:
    break;
  }
  /*Receive x,y,z values*/

}
void i2c_setSlave(){
  //Serial.setDebugOutput(true);
  Wire.onReceive(i2c_onReceive);
  Wire.onRequest(i2c_onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
}

