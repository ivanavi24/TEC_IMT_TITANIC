import numpy as np



def getCameraIntrinsicInverse(Mint):
    '''
        Get camera inverse matrix from a camera intrinsic Matrix
        Mint= 
        [
            fx  0   cx    
            0   fy  cy
            0   0   1        
        ]
        MintInv=
        [
            1/fx  0     -cx/fx
             0    1/fy   -cy/fy
             0     0        1

        ]
    '''
    MintInverse=np.zeros_like(Mint)
    MintInverse[0,0]=1/Mint[0,0]   #1/fx 
    MintInverse[0,2]=-Mint[0,2]/Mint[0,0] #-cx/fx
    MintInverse[1,1]=1/Mint[1,1] #1/fy
    MintInverse[1,2]=-Mint[1,2]/Mint[1,1] #-cy/fy
    MintInverse[2,2]=1                 #1
    return MintInverse
def getCameraCoordsScaled(pixelCoords:np.array,MintInverse:np.array):
    '''
        Get augmented camera coords (Xcam',Ycam')=(Xcam/Zcam,Ycam/Zcam) 
        from the pixel values (u,v) and the camera intrinsinc Matrix

        Return: Xcam_au,Ycam_au
    '''
    pixelCoordsH=np.array(pixelCoords+(1,))
    cameraCoordsAugmented=np.matmul(MintInverse,pixelCoordsH)
    Xcam_au=cameraCoordsAugmented[0]
    Ycam_au=cameraCoordsAugmented[1]
    return Xcam_au,Ycam_au

def getWorldCoords(RotationMatrix,TranslationVector,CameraCoordsAugmented,Zw=0):
    '''Get Woorld Coordintes given a point u',v' in the image and the camera '''


    r1,r2,r3,r4,r5,r6,r7,r8,r9=RotationMatrix
    Tx,Ty,Tz=TranslationVector
    k1,k2=CameraCoordsAugmented

    h1=r3*Zw+Tx
    h2=r6*Zw+Ty
    h3=r9*Zw+Tz

    g1=r4*r8-r5*r7
    g2=r2*r7-r1*r8
    g3=r1*r5-r2*r4

    g4=r8*k2-r5
    g5=r2-r8*k1
    g6=r5*k1-r2*k2

    g7=r4*-r7*k2
    g8=r7*k1-r1
    g9=r1*k2-r4*k1

    Zc=g1*h1+g2*h2+g3*h3 

    Xw=g4*h1+g5*h2+g6*h3

    Yw=g7*h1+g8*h2+g9*h3

    det=k1*(r4*r8-r5*r7)+r1*(-r8*k2+r5)-r2*(-r7*k2+r4)
    #r4r8k1 - r5 r7 k1 - r1 r8 k2 +r1 r5 +r2 r7 k2-r2r4

    return Xw/det, Yw/det #Florecita
def getWorldCoords2(RotationMatrix,TranslationVector,CameraCoordsAugmented,Zw=0):
    '''Get Woorld Coordintes given a point u',v' in the image and the camera '''


    r1,r2,r3,r4,r5,r6,r7,r8,r9=RotationMatrix
    Tx,Ty,Tz=TranslationVector
    k1,k2=CameraCoordsAugmented

    A=np.array([[k1,-r1,-r2],[k2,-r4,-r5],[1,-r7,-r8]])
    B=np.array([[r3*Zw+Tx],[r6*Zw+Ty],[r9*Zw+Tz]])
    X=np.matmul(np.linalg.inv(A),B)
    Zcam,Xw, Yw=tuple(X.flatten())
    return Zcam,Xw, Yw #Florecita


def denavitTransformation(theta:float,trans_d:float,trans_a:float,alpha:float,dimension:float=4):
    '''
        Get denavit hartenberg transformation given theta, d, a and alpha
        warnings: 
        a) angle must be in radians
        b) distances preferable in meters[m] 

    '''
    transform=np.zeros(shape=(dimension,dimension))
    
    transform[0,0]=np.cos(theta)  
    transform[0,1]=-np.sin(theta)*np.cos(alpha)
    transform[0,2]=np.sin(theta)*np.sin(alpha)
    transform[0,3]=trans_a*np.cos(theta)

    transform[1,0]=np.sin(theta)
    transform[1,1]=np.cos(theta)*np.cos(alpha)
    transform[1,2]=-np.cos(theta)*np.sin(alpha)
    transform[1,3]=trans_a*np.sin(theta)

    transform[2,1]=np.sin(alpha)
    transform[2,2]=np.cos(alpha)
    transform[2,3]=trans_d

    transform[3,3]=1

    return transform
#General Denavit Hartenberg representation DH does not follow H-1=H(t)
def manipulatorTransformation(DHvars,num=4):  #2.8 cm   1 cm
    #
    transformation=np.identity(n=num)
    
    for (theta,d,a,alpha) in DHvars:
        
        T_n=denavitTransformation(theta,d,a,alpha)
        
        transformation=np.matmul(transformation,T_n)

    #a1,alpha1,d1,theta1=fisrt
    #T0_1=denavitTransformation(theta1,d1,a1,alpha1)
    #a2=1
    #alpha2=1
    #d2=1
    #theta2=1
    #T0_1=denavitTransformation(theta2,d2,a2,alpha2)
    return transformation
def panTiltTransformInverse(kinematicMatrix):
    return np.linalg.inv(kinematicMatrix)

def compWorld2CameraMatrix():
    pass
