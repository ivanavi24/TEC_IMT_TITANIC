"""
    Check if pixel to world transformation has sense
"""
print(__doc__)

import sys
import os


separator = "\\"
librariesFolderName  = 'libraries'
calibrationFolderName = 'calibration'
calibrationFileName = 'logitecCalib1'
calibrationFileSuffix = '.txt'
pythonScriptPath = os.path.abspath(__file__)
transformationFolder = os.path.dirname(pythonScriptPath)
titanicFolder = os.path.dirname(transformationFolder)
librariesFolder = os.path.join(titanicFolder,librariesFolderName)
calibrationFolder = os.path.join(titanicFolder,calibrationFolderName)
sys.path.append(librariesFolder)

import cv2 as cv
import numpy as np
MintIntrinsic=np.loadtxt(calibrationFolder+separator+calibrationFileName+calibrationFileSuffix) #Load intrinsic param values

#import libraries.image2world as im2wld
from image2world import *
def click_event(event,x,y,flags,params):
    global xcoor,ycoor,myflag
    if event==cv.EVENT_LBUTTONDOWN:
        myflag=1
        xcoor=x
        ycoor=y
def sliderChanged( value):
    global zval,myflag
    zval =value/100
    myflag=1
    pass


webcamNum=int(input('Enter the number of webcam to open (0 Embedded, 1 external webcam): '))
criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,0.001)
webCam=cv.VideoCapture(webcamNum)



result,image=webCam.read()
cv.imshow("Imagen",image)
#cv.createTrackbar('z distance',"Imagen",0,100,sliderChanged)

pi=3.1415926535
laptop2Tilt=0.02 #[m]
desk2Floor=0.805

firstJoint=[pi/2,desk2Floor,0,-pi/2,]#(theta1,d1,a1,alpha1,)
secondJoint=[-pi/2,0,laptop2Tilt,-pi/2,]#(theta2,d2,a2,alpha2,)
thirdJoint=[pi/2,0,0,0,]#(theta3,d3,a3,alpha3,) Extra rotation to adapt to camera coordinate frame
params=[firstJoint,secondJoint,thirdJoint]

xcoor=0
ycoor=0
zval=0

#MintIntrinsic[0,2]=240
#MintIntrinsic[1,2]=320
cv.setMouseCallback("Imagen",click_event)
MintIntInv=getCameraIntrinsicInverse(MintIntrinsic)
myflag=0
#Check Results
while (webCam.isOpened()):
    result,image=webCam.read() 
    if myflag:
        theta1=float(input('Enter theta1(camera inclination horizontal [degrees]): '))
        theta2=float(input('Enter theta2(camera inclination [degrees]): '))
        
        params[0][0]=(theta1*pi/180)+pi/2
        params[1][0]=(theta2*pi/180)-pi/2
        externalMatrix=panTiltTransformInverse(manipulatorTransformation(params))
        rotationMatrix=tuple(externalMatrix[:-1,:-1].flatten())
        translationVector=tuple(externalMatrix[:-1,-1].flatten())

        cameraCordsAug=getCameraCoordsScaled((xcoor,ycoor),MintIntInv) 
        Zcam,Xreal, Yreal=getWorldCoords2(rotationMatrix,translationVector,cameraCordsAug,Zw=0)
        print(' EXPERIMENTAL RESULTS\n')
        print(f'X pixel:{xcoor}  Y pixel:{ycoor} zcam: {Zcam}')
        print(f'X world:{Xreal}  Y world:{Yreal} Z world: {zval}')
        Xreal, Yreal=getWorldCoords(rotationMatrix,translationVector,cameraCordsAug,Zw=0)
        print(f'X world:{Xreal}  Y world:{Yreal} Z world: {zval}')
        myflag=0
        endflag=float(input('Enter 0 to leave program: '))
        if not endflag:
            break
    if cv.waitKey(27) & 0xFF ==ord('z'):
        break
    cv.imshow("Imagen",image)