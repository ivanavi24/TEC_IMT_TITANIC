# i2ctest.py
# A brief demonstration of the Raspberry Pi I2C interface, using the Sparkfun
# Pi Wedge breakout board and a SparkFun MCP4725 breakout board:
# https://www.sparkfun.com/products/8736

import smbus
import time



def stature16bit(data,minVal,maxVal,numBIts=16):
    '''
        Escale value to a 16 bit unsigned representation
    '''
    normalizedData=(data-minVal)/(maxVal-minVal)
    if normalizedData>=1:
        return 2**(numBIts)-1
    if normalizedData<=0:
        return 0
    final16bitVvalue=int(normalizedData*(2**(numBIts)))
    print(final16bitVvalue)
    return final16bitVvalue
    


def sendI2Cint16(i2cBus,address,uint16number):
    '''
        CUstom function to write a 16 bit unsigned value to I2C BUffer
    '''
    if((uint16number<2**16)&(uint16number>0)&isinstance(uint16number,int)):
        i2cBus.write_i2c_block_data(address,0x049,[uint16number>>8, uint16number& 0XFF])
        return 1
    return 0

def sendI2Cint16Block(i2cBus,address,dataList,formatList,startIdentifier,endIdentifier):
    '''
        CUstom function to write a 16 bit unsigned value to I2C BUffer
    '''
    messageList=[startIdentifier]
    for data,(minVal,maxVal,numBits) in zip(dataList,formatList):
        number=stature16bit(data,minVal,maxVal,16)
        messageList.append(number>>8)
        messageList.append(number& 0xFF)
    messageList.append(endIdentifier)
    
    #if((uint16number<2**16)&(uint16number>0)&isinstance(uint16number,int)):
    print(messageList)
    i2cBus.write_i2c_block_data(address,0x049,messageList)
    return 1
    #return 0

def receiveI2Cbyte():
    pass
def initializeI2Cbus(master:bool=True,channel:int=1,slaveAddress:int=0x25):
    if master:
        bus = smbus.SMBus(channel)
    time.sleep(1)
    return bus
