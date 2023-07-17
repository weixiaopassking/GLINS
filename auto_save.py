import os
import sys




if __name__ == '__main__':
    write_log='update'
    param_num=len(sys.argv)
    if len==2 :
        write_log = sys.argv[1]
    os.system('git add .')
    os.system('git commit -m'+write_log)
    print(write_log)
    os.system('git push')
    if len==3 :
         time_delay=sys.argv[2]
         os.system('shutdown -h %d' % (time_delay))
 
