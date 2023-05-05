print('Script to open video Stream')

# C:\Users\POJ1GA\.conda\envs\ogt2\python.exe C:\Users\POJ1GA\Documents\TEC\computerVision\Titanic\openWebcams.py
import cv2 as cv
import numpy as np
from image2world import *


webCam=cv.VideoCapture(1)

result,image=webCam.read() 
if result:
    print(f'Camera resolution is: {image.shape}')

#Check Results
while (webCam.isOpened()):
    result,image=webCam.read() 
    if cv.waitKey(27) & 0xFF ==ord('z'):
        break
    cv.imshow("Imagen",image)