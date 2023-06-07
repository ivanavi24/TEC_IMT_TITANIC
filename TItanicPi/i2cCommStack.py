import i2c2 as i2c

slave=int(input("ENter the slave adress: "))
x=float(input('Enter the X coordinate to be transmitted: '))
y=float(input('Enter the Y coordinate to be transmitted: '))
z=float(input('Enter the Z coordinate to be transmitted: '))
i2cBUs=i2c.initializeI2Cbus()
startId=0x55
endId=0xAA
dataL=[x,y,z]
formatL=[(-(2**15-1)/1000,(2**15-1)/1000,16),]*len(dataL)
i2c.sendI2Cint16Block(i2cBUs,slave,dataL,formatL,startId,endId)