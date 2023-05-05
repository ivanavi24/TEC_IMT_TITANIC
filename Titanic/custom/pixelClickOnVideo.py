import cv2 as cv
import numpy as np

#C:\Users\POJ1GA\.conda\envs\ogt2\python.exe C:\Users\POJ1GA\Documents\TEC\Titanic\generateCB.py

def click_event(event,x,y,flags,params):
    global xcoor,ycoor,myflag
    if event==cv.EVENT_LBUTTONDOWN:
        xcoor=x
        ycoor=y
        myflag=1

webCam=cv.VideoCapture(0)
chessBoardShape=(13,9) #COLS ROWS
result,image=webCam.read()
xcoor=0
ycoor=0
myflag=0
while (webCam.isOpened()):
    result,image=webCam.read()
    if result:
        cv.imshow("Imagen",image)
        cv.setMouseCallback("Imagen",click_event)
        if cv.waitKey(27) & 0xFF ==ord('z'):
            break
        if myflag:
            print(f'X:{xcoor}  Y:{ycoor}')
            myflag=0
