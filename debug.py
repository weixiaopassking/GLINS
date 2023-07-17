#-- coding:UTF-8 --
import os


exec_path='bin/test_node'

class bcolor(object):
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'



#example
# print(bcolor.WARNING + 'WARNING: start httpd failed' + bcolor.ENDC)
# print(bcolor.OKGREEN + 'starting......' + bcolor.ENDC)
# print(bcolor.OKBLUE + 'starting......' + bcolor.ENDC)
# print(bcolor.FAIL + 'starting......' + bcolor.ENDC)
# print(bcolor.HEADER + 'starting......' + bcolor.ENDC)


#编译
build_code = os.system('cmake -j8 -S . -B build && cmake --build  build')
if build_code==0:
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'编译成功' + bcolor.ENDC)
else :
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.FAIL +'编译失败' + bcolor.ENDC)
#运行
print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'运行启动......' + bcolor.ENDC)
run_code=os.system(exec_path)
if run_code==0:
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'运行正常' + bcolor.ENDC)
else :
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.FAIL +'运行错误' + bcolor.ENDC)

