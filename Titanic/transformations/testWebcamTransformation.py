"""
    Test Webcam(laptop) Transformation from world coordinate frame to end efector an
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
laptop2Tilt=0.226 #[m]
desk2Floor=0.805
firstJoint=[pi/2,desk2Floor,0,-pi/2,]#(theta1,d1,a1,alpha1,)
secondJoint=[-pi/2,0,laptop2Tilt,-pi/2,]#(theta2,d2,a2,alpha2,)
thirdJoint=[pi/2,0,0,0,]#(theta3,d3,a3,alpha3,) Extra rotation to adapt to camera coordinate frame
params=[firstJoint,secondJoint,thirdJoint]


while True:
    theta2=float(input('Enter theta2: [degrees] '))
    params[1][0]=(theta2/180*pi)-pi/2
    transform=manipulatorTransformation(params)
    print('The origin of end effector in WORLD COORDS is: \n')
    print(transform[:,-1])
    print('The origin of World in EF COORDS is: \n')
    print(panTiltTransformInverse(transform)[:,-1])
    
    endFlag=int(input('Enter number 0 if you want to end the program: '))
    if not endFlag:
        break

np.savetxt('myArray',np.random.random(size=(3,3)))