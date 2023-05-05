print('Script to calibrate camera')

# C:\Users\POJ1GA\.conda\envs\ogt2\python.exe C:\Users\POJ1GA\Documents\TEC\Titanic\cbDetect.py
import cv2 as cv

criteria=(cv.TERM_CRITERIA_EPS+cv.TERM_CRITERIA_MAX_ITER,30,0,0.001)
chessBoardShape=(3,3)
image=cv.imread('capture.png')
cv.imshow("Captura",image)
cv.waitKey(0)
grayCaptured=cv.cvtColor(image,cv.COLOR_BGR2GRAY)
ret,corners=cv.findChessboardCorners(grayCaptured,chessBoardShape,cv.CALIB_CB_ADAPTIVE_THRESH+cv.CALIB_CB_NORMALIZE_IMAGE+cv.CALIB_CB_FAST_CHECK    )
cv.imshow("Captura",grayCaptured)
cv.waitKey(0)
for col in range(3,20,1):
    for row in range (3,20,1):
        ret,corners=cv.findChessboardCorners(grayCaptured,(col,row),cv.CALIB_CB_ADAPTIVE_THRESH+cv.CALIB_CB_NORMALIZE_IMAGE+cv.CALIB_CB_FAST_CHECK    )
        if ret:
            print(f'Columna: {col}  Fila: {row}')
            #cornerStruct=cv.cornerSubPix(grayCaptured,corners,(11,11),(-1,-1),criteria)
            cv.drawChessboardCorners(image,(col,row),corners,ret)
            cv.imshow("Captura con tablero",image)
            cv.waitKey(0)
        else:
            pass
            #print("No squares of chessboard were found")

cv.destroyAllWindows()