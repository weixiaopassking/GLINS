import os
import sys

if __name__ == '__main__':
    param_num=len(sys.argv)
    print(param_num)
    write_log="update"
    if param_num==2 :
        write_log = sys.argv[1]
        print(write_log)
    os.system('git add .')
    os.system('git commit -m %s' % (write_log))
    print(write_log)
    os.system('git push')
    if param_num==3 :
         time_delay=sys.argv[2]
         print(time_delay)
         os.system('shutdown -h'+ time_delay)
 
