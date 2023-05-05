"""
    Test pan Tilt Transformation from world coordinate frame to end efector an
    vice versa (Script relys on a 2 degree manipulador)
"""
    
print(__doc__)

import sys
import os
librariesFolderName  = 'libraries'
calibrationFileSuffix = '.txt'
pythonScriptPath = os.path.abspath(__file__)
transformationFolder = os.path.dirname(pythonScriptPath)
titanicFolder = os.path.dirname(transformationFolder)
librariesFolder = os.path.join(titanicFolder,librariesFolderName)

sys.path.append(librariesFolder)

from image2world import manipulatorTransformation,panTiltTransformInverse
import numpy as np
pi=3.1415926535

firstJoint=[0,0,0,-pi/2,]#(theta1,d1,a1,alpha1,)
secondJoint=[0,0,5,pi/2,]#(theta2,d2,a2,alpha2,)
thirdJoint=[-pi/2,0,0,-pi/2,]#(theta3,d3,a3,alpha3,) Extra rotation to adapt to camera coordinate frame
params=[firstJoint,secondJoint,thirdJoint]

while True:
    theta1=float(input('Enter theta1: '))   
    theta2=float(input('Enter theta2: '))
    params[0][0]=theta1+pi/2
    params[1][0]=theta2
    transform=manipulatorTransformation(params)
    print('The origin of end effector is: \n')
    print(transform[:,-1])
    print('The origin of World in EF coords is: \n')
    print(panTiltTransformInverse(transform)[:,-1])
    
    endFlag=int(input('Enter number 0 if you want to end the program: '))
    if not endFlag:
        break

np.savetxt('myArray',np.random.random(size=(3,3)))