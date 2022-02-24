#!/usr/bin/python3
# File name   : setup.py for WAVEGO
# Date        : 2022/1/5

import os
import time
import re

curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)

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

for x in range(1,4):
	if os.system("sudo apt update") == 0:
		break

for x in range(1,4):
	if os.system("sudo apt -y dist-upgrade") == 0:
		break

for x in range(1,4):
	if os.system("sudo apt clean") == 0:
		break

for x in range(1,4):
	if os.system("sudo pip3 install -U pip") == 0:
		break

for x in range(1,4):
	if os.system("sudo apt-get install -y python-dev python3-pip libfreetype6-dev libjpeg-dev build-essential") == 0:
		break

for x in range(1,4):
	if os.system("sudo apt-get install -y i2c-tools") == 0:
		break

for x in range(1,4):
	if os.system("sudo apt-get install -y python3-smbus") == 0:
		break

for x in range(1,4):
	if os.system("sudo pip3 install pyserial") == 0:
		break
	elif os.system("sudo pip3 install -i http://pypi.douban.com/simple/ --trusted-host=pypi.douban.com/simple pyserial") == 0:
		break

for x in range(1,4):
	if os.system("sudo pip3 install flask") == 0:
		break
	elif os.system("sudo pip3 install -i http://pypi.douban.com/simple/ --trusted-host=pypi.douban.com/simple flask") == 0:
		break

for x in range(1,4):
	if os.system("sudo pip3 install flask_cors") == 0:
		break
	elif os.system("sudo pip3 install -i http://pypi.douban.com/simple/ --trusted-host=pypi.douban.com/simple flask_cors") == 0:
		break

for x in range(1,4):
	if os.system("sudo pip3 install websockets") == 0:
		break
	elif os.system("sudo pip3 install -i http://pypi.douban.com/simple/ --trusted-host=pypi.douban.com/simple websockets") == 0:
		break
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


CMDLINE_FILE = open('/boot/cmdline.txt', 'r')
OLD_LINES = CMDLINE_FILE.readlines()
CMDLINE_FILE.close()

CMDLINE_FILE = open('/boot/cmdline.txt', 'w+')
for EACH_LINE in OLD_LINES:
	NEW_LINES = re.sub('console=serial0,115200', '', EACH_LINE)
	CMDLINE_FILE.writelines(NEW_LINES)

CMDLINE_FILE.close()


for x in range(1,4):
	if os.system("sudo pip3 install opencv-contrib-python==3.4.11.45") == 0:
		break
	elif os.system("sudo pip3 install -i http://pypi.douban.com/simple/ --trusted-host=pypi.douban.com/simple opencv-contrib-python==3.4.11.45") == 0:
		break



for x in range(1,4):
	if os.system("sudo pip3 uninstall -y numpy") == 0:
		break

for x in range(1,4):
	if os.system("sudo pip3 install numpy==1.21") == 0:
		break
	elif os.system("sudo pip3 install -i http://pypi.douban.com/simple/ --trusted-host=pypi.douban.com/simple numpy==1.21") == 0:
		break

for x in range(1,4):
	if os.system("sudo apt-get -y install libhdf5-dev libhdf5-serial-dev libatlas-base-dev libjasper-dev") == 0:
		break

for x in range(1,4):
	if os.system("sudo pip3 install imutils zmq pybase64 psutil") == 0:
		break
	elif os.system("sudo pip3 install -i http://pypi.douban.com/simple/ --trusted-host=pypi.douban.com/simple imutils zmq pybase64 psutil") == 0:
		break

for x in range(1,4):
	if os.system("sudo apt-get install -y util-linux procps hostapd iproute2 iw haveged dnsmasq") == 0:
		break
        
for x in range(1,4):
	if os.system("cd " + thisPath + " && cd .. && sudo git clone https://github.com/oblique/create_ap") == 0:
		break

try:
	os.system("cd " + thisPath + " && cd .. && cd create_ap && sudo make install")
except:
	pass

replace_num('/etc/rc.local','exit 0','cd '+thisPath+' && sudo python3 webServer.py &\nexit 0')

print('Completed!')

os.system("sudo reboot")