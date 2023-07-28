#-- coding:UTF-8 --
import os
import sys


class bcolor(object):
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'



#编译
build_status = os.system('catkin_make  -j16')
if build_status==0:
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'编译成功' + bcolor.ENDC)
else :
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.FAIL +'编译失败' + bcolor.ENDC)
    quit()

run_status=os.system('rosrun quadrotor_source test_pipe')
if run_status==0:
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'运行正常' + bcolor.ENDC)
else :
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.FAIL +'运行错误' + bcolor.ENDC)
    quit()




