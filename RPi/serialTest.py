#!/usr/bin/python3
# File name   : serialTest.py
# Date        : 2022/1/5

import re
import os

def replace_num(file,initial,new_num):  
    newline=""
    str_num=str(new_num)
    with open(file,"r") as f:
        for line in f.readlines():
            if(line.find(initial) == 0):
                line = (str_num+'\n')
            newline += line
    with open(file,"w") as f:
        f.writelines(newline)


CMDLINE_FILE = open('/boot/cmdline.txt', 'r')
OLD_LINES = CMDLINE_FILE.readlines()
CMDLINE_FILE.close()

CMDLINE_FILE = open('/boot/cmdline.txt', 'w+')
for EACH_LINE in OLD_LINES:
	NEW_LINES = re.sub('console=serial0,115200', '', EACH_LINE)
	CMDLINE_FILE.writelines(NEW_LINES)

CMDLINE_FILE.close()

try:
	replace_num("/boot/config.txt",'[all]','[all]\nenable_uart=1\ngpu_mem=128')
except:
	print('try again')

try:
	replace_num("/boot/config.txt",'camera_auto_detect=1','#camera_auto_detect=1\nstart_x=1')
except:
	print('try again')

try:
	replace_num("/boot/config.txt",'camera_auto_detect=1','#camera_auto_detect=1')
except:
	print('try again')