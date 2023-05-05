print('Script to calibrate camera')

# C:\Users\POJ1GA\.conda\envs\ogt2\python.exe C:\Users\POJ1GA\Documents\TEC\computerVision\Titanic\ideastest2.py

import sys
import os
sys.path.insert(0,r'C:\Users\POJ1GA\Documents\TEC\computerVision\Titanic\libraries')
print(os.path.dirname(os.path.abspath(__file__)))
print(os.path.dirname((os.path.dirname(os.path.abspath(__file__)))))
print(os.path.dirname(os.path.dirname((os.path.dirname(os.path.abspath(__file__))))))

print(os.path.abspath('.'))