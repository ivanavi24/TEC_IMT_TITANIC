import i2c2
import time

Bus=i2c2.initializeI2Cbus(True,1,0x25) 
address=0x55
myNUmber=i2c2.stature16bit(387,maxVal=1000,minVal=0,numBIts=16)
i2c2.sendI2Cint16(Bus,address,39000)
#print(myNUmber)
#print(f'{myNUmber:b}')
#print(f'{myNUmber>>8:b}')
#print(f'{myNUmber&0x00FF:b}')
for i in range(0,11):
    print(i*6000)
    i2c2.sendI2Cint16(Bus,address,i*6000)
    time.sleep(1)