#-- coding:UTF-8 --
import os
import sys

HEADER='\033[1;34m'  
SUCCESS='\033[1;32m'  
FAIL = '\033[1;31m'
RESET = '\033[0m'

os.system('catkin_make  clean')
STATUS = os.system('catkin_make  -j${nproc}')
if STATUS==0:
    print('\n'+HEADER+ 'AlkaidQuadrotor$ '+SUCCESS +'compile success' + RESET)
else :
    print('\n'+HEADER + 'AlkaidQuadrotor$ '+FAIL +'compile failed' + RESET+'\n')
    quit()

os.system('tree -L 2  ./devel/lib/')


