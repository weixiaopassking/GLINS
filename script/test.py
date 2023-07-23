#-- coding:UTF-8 --
import os
import sys

build_code = os.system('cd build && rm -rf * && cmake -j16 .. && make -j16 ')