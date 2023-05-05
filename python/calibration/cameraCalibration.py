print('Script to calibrate camera')

# C:\Users\POJ1GA\.conda\envs\ogt2\python.exe C:\Users\POJ1GA\Documents\TEC\computerVision\Titanic\cameraCalibration.py
import cv2 as cv
import numpy as np


calibrationName=str(input('Enter the name of the file to store the calibration: '))
webcamNum=int(input('Enter the number of webcam to open (0 Embedded, 1 external webcam): '))
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


criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,0.001)
webCam=cv.VideoCapture(webcamNum)
chessBoardShape=(6,8) #COLS ROWS



xcoo=0
ycoor=0
myflag=0
squareLength=2/100# [M]

#Initialize Object Points

objP=np.zeros(shape=(chessBoardShape[0]*chessBoardShape[1],3),dtype=np.float32)
objP[:,:2]=np.mgrid[0:chessBoardShape[0],0:chessBoardShape[1]].T.reshape(-1,2)*squareLength# x Rows, y Cols


imgPoints=[]
objectPoints=[]

result,image=webCam.read()
if result:
    resolution=image.shape[:2]
    print(resolution)
#Capture image form video stream when z keyboard is pressed
calibNumer=5



for caliImg in range(1,calibNumer+1):
    while (webCam.isOpened()):
        flag=False
        result,image=webCam.read()
        if result:
            grayCaptured=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
            ret,corners=cv.findChessboardCorners(grayCaptured,chessBoardShape, None)#None
            if ret:
                cornerStruct=cv.cornerSubPix(grayCaptured,corners,(11,11),(-1,-1),criteria)
                cv.drawChessboardCorners(image,chessBoardShape,cornerStruct,ret)
                flag=True
        if (cv.waitKey(27) & 0xFF==ord('z'))&flag:

            break

        cv.imshow("Captura",image)
    print(corners)
    cv.destroyWindow("Captura")
    cv.imshow("Calib img "+str(caliImg),image)
    cv.waitKey(0)
    cv.destroyWindow("Calib img "+str(caliImg))
    imgPoints.append(corners)
    objectPoints.append(objP)
print(f'El numero de total de muestras fue {len(imgPoints)}')
print(f'El numero de total de muestras fue {len(objectPoints)}')
ret,mtx,dist,rves,tvecs=cv.calibrateCamera(objectPoints,imgPoints,resolution,None,None)
print(ret)
print(mtx)
print(dist)
print(rves)
print(tvecs)
np.savetxt(calibrationName+'.txt',mtx)
cv.destroyAllWindows()


cx=mtx[0,2]
cy=mtx[1,2]
fx=mtx[0,0]
fy=mtx[1,1]
zval=0


result,image=webCam.read()
cv.imshow("Imagen",image)
cv.createTrackbar('z distance',"Imagen",0,100,sliderChanged)


xcoor=0
ycoor=0
zval=0
#Check Results
while (webCam.isOpened()):
    result,image=webCam.read()
    
    cv.setMouseCallback("Imagen",click_event)
    if myflag:
        Xreal=((xcoor-cx)/fx)*zval
        Yreal=((ycoor-cy)/fy)*zval
        print(f'X pixel:{xcoor}  Y pixel:{ycoor} ')
        print(f'X world:{Xreal}  Y world:{Yreal} Z world: {zval}')
        myflag=0
    if cv.waitKey(27) & 0xFF ==ord('z'):
        break
    cv.imshow("Imagen",image)