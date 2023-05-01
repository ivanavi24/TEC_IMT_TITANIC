#include "Wire.h"
#include "joint1_config.h"
#include "joint2_config.h"
#include "joint3_config.h"
#include  "i2c_config.h"
#include "Crane3dof.h"


/* Kinematic variable definition*/

#define origin2waterDistance            30.4







#define PI                             3.14159265

#define PID_INTERVAL 5
uint32_t i = 0;
long time_now=millis();
long time_last=millis();




Crane3dof titanicCrane;
void onRequest(){
  
  Wire.print(i++);
  Wire.print(" Packets.");
  Serial.println("onRequest");
}
int readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
  return bComposed;
}
void onReceive(int len){
  /*Receive x,y,z values*/
  if (len==8){
    int tempByte = Wire.read();
    if (tempByte == START_COMMAND){
      float xref=(float(readUint16data())- pow(2,15))/1000;
      float yref=(float(readUint16data())- pow(2,15))/1000;
      float zref=(float(readUint16data())- pow(2,15))/1000;
      tempByte = Wire.read();
      if(tempByte == END_COMMAND){
        titanicCrane.inverse_kinematics( xref,  yref,  zref);
        titanicCrane.setTargetJoints(); 
      }      
    }
    else{
      while(Wire.available()){
        Wire.read();
      }
    }
    
    
  }
  /*Command with gripper action*/


  /*Command with just arm action*/
  Serial.printf("onReceive[%d]: ", len);
  
  while(Wire.available()){
    Serial.write(Wire.read());
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
  Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  

}
/*Main Loop*/
void loop() {
  time_now=millis();
  if(abs(time_now-time_last)>=PID_INTERVAL or (time_last > time_now)){
    /*Control action*/  
    titanicCrane.reachPosition(time_now-time_last);
    time_last=millis();
  }

}
