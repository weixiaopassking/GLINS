import os
import sys

if __name__ == '__main__':
    param_num=len(sys.argv)
    print(param_num)
    write_log=''
    if len==2 :
        write_log = sys.argv[1]
    os.system('git add .')
    os.system('git commit -m "update"')
    print(write_log)
    os.system('git push')
    if len==3 :
         time_delay=sys.argv[2]
         os.system('shutdown -h %d' % (time_delay))
 
