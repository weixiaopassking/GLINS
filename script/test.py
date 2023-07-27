#-- coding:UTF-8 --
import os
import sys

os.system('catkin_make  -j16')
os.system('rosrun quadrotor_localization vis')
