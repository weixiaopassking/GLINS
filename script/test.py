#-- coding:UTF-8 --
import os
import sys

build_code = os.system('cd build && cmake -j16 .. && make -j16')