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

build_code = os.system('cmake -j8 -S . -B build && cmake --build  build')
if build_code==0:
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'程序编译成功' + bcolor.ENDC)
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'程序执行......' + bcolor.ENDC)
    run_code=os.system(exec_path)
    if run_code==0:
            print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.OKGREEN +'程序正常结束' + bcolor.ENDC)
else :
    print(bcolor.OKBLUE + 'AlkaidQuadrotor$ '+bcolor.FAIL +'程序编译失败' + bcolor.ENDC)

#example
# print(bcolor.WARNING + 'WARNING: start httpd failed' + bcolor.ENDC)
# print(bcolor.OKGREEN + 'starting......' + bcolor.ENDC)
# print(bcolor.OKBLUE + 'starting......' + bcolor.ENDC)
# print(bcolor.FAIL + 'starting......' + bcolor.ENDC)
# print(bcolor.HEADER + 'starting......' + bcolor.ENDC)