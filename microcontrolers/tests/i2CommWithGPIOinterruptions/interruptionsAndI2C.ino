
#include "Wire.h"

#define I2C_DEV_ADDR 0x55


#define START_COMMAND                   0x55 /* binary ->  0101 0101*/
#define END_COMMAND                     0xAA /* binary ->  1010 1010*/


long int timeNow=0;
long int timeLast=0;
int readUint16data(){
  int bMSB=Wire.read();
  int bLSB=Wire.read();
  int bComposed = (bMSB<<8) + bLSB ; // Reading is in cmm -32,678
  return bComposed;
  
}
void onReceive(int len){
  
  /*Receive x,y,z values*/
  Serial.printf("Received con call %d: ", len);
  Serial.print("  Starting delay, hope to see the LED not blinking......");
  delay(10000);
  Serial.print("  FINISH");
  Serial.println();
}
class miClass{
  private:
    int variable = 0;
  public:
    void increaseVariable(){
      variable++;
    }
    int getVariable(){
      return variable;
    }
};
miClass obj1;
void IRAM_ATTR isr(){
  static int val =0;
  digitalWrite(2,val=!val);
}
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Wire.onReceive(onReceive);
 // Wire.onRequest(onRequest);
  Wire.begin((uint8_t)I2C_DEV_ADDR);
  Serial.begin(115200);
  pinMode(18,INPUT  );
  pinMode(2,OUTPUT);
  attachInterrupt(18, isr, FALLING);
  

}
/*Main Loop*/
void loop() {

}
