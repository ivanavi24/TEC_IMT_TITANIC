
#include "Crane3dof.h"

Crane3dof titanicCrane;
int maxVal=10;
int minVal=0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  float pwmT[3]={5,8,3};
  titanicCrane.moveMotors(pwmT,minVal , maxVal, 8);
  delay(1000);
  /*for (int i=minVal;i<=maxVal;i++){
    float pwmT[3]={i,(maxVal+minVal)/2,maxVal-i};
    titanicCrane.moveMotors(pwmT,minVal , maxVal, 16);
    delay(1000);
  }
  */
}
