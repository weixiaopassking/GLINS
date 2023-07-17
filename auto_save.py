import os
import sys


write_log='update'

if __name__ == '__main__':

    param_num=len(sys.argv)
    if len==1 :
        write_log = sys.argv[1]


    os.system('git add .')
    os.system('git commit -m %s' % (write_log))
    os.system('git push')
    if len==2 :
         time_delay=sys.argv[2]
         os.system('shutdown -h %d' % (time_delay))
 
