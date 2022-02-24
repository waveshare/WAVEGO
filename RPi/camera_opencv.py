import os
import cv2
from base_camera import BaseCamera
import numpy as np
import robot
import datetime
import time
import threading
import imutils

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

faceCascade = cv2.CascadeClassifier(thisPath + '/haarcascade_frontalface_default.xml')

upperGlobalIP = 'UPPER IP'

linePos_1 = 440
linePos_2 = 380
lineColorSet = 255
frameRender = 1
findLineError = 20

colorUpper = np.array([44, 255, 255])
colorLower = np.array([24, 100, 100])

speedMove = 100



class CVThread(threading.Thread):
    font = cv2.FONT_HERSHEY_SIMPLEX

    cameraDiagonalW = 64
    cameraDiagonalH = 48
    videoW = 640
    videoH = 480
    tor = 27
    aspd = 0.005


    def __init__(self, *args, **kwargs):
        self.CVThreading = 0
        self.CVMode = 'none'
        self.imgCV = None
        self.faces = None

        self.mov_x = None
        self.mov_y = None
        self.mov_w = None
        self.mov_h = None

        self.radius = 0
        self.box_x = None
        self.box_y = None
        self.drawing = 0

        self.findColorDetection = 0

        self.left_Pos1 = None
        self.right_Pos1 = None
        self.center_Pos1 = None

        self.left_Pos2 = None
        self.right_Pos2 = None
        self.center_Pos2 = None

        self.center = None

        super(CVThread, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()
        self.__flag.clear()

        self.avg = None
        self.motionCounter = 0
        self.lastMovtionCaptured = datetime.datetime.now()
        self.frameDelta = None
        self.thresh = None
        self.cnts = None

        self.CVCommand = 'forward'


    def mode(self, invar, imgInput):
        self.CVMode = invar
        self.imgCV = imgInput
        self.resume()


    def elementDraw(self,imgInput):
        if self.CVMode == 'none':
            pass

        elif self.CVMode == 'faceDetection':
            if len(self.faces):
                if len(self.faces) == 1:
                    cv2.putText(imgInput,'1 Face Detected',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                else:
                    cv2.putText(imgInput,'%d Faces Detected'%len(self.faces),(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
            else:
                cv2.putText(imgInput,'Face Detecting',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
            for (x,y,w,h) in self.faces:
                cv2.rectangle(imgInput,(x,y),(x+w,y+h),(64,128,255),2)

        elif self.CVMode == 'findColor':
            if self.findColorDetection:
                cv2.putText(imgInput,'Target Detected',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                self.drawing = 1
            else:
                cv2.putText(imgInput,'Target Detecting',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                self.drawing = 0

            if self.radius > 10 and self.drawing:
                cv2.rectangle(imgInput,(int(self.box_x-self.radius),int(self.box_y+self.radius)),(int(self.box_x+self.radius),int(self.box_y-self.radius)),(255,255,255),1)

        elif self.CVMode == 'findlineCV':
            if frameRender:
                imgInput = cv2.cvtColor(imgInput, cv2.COLOR_BGR2GRAY)
                retval_bw, imgInput =  cv2.threshold(imgInput, 0, 255, cv2.THRESH_OTSU)
                imgInput = cv2.erode(imgInput, None, iterations=6)
            try:
                if lineColorSet == 255:
                    cv2.putText(imgInput,('Following White Line'),(30,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
                    cv2.putText(imgInput,('Following White Line'),(230,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,0),1,cv2.LINE_AA)
                else:
                    cv2.putText(imgInput,('Following Black Line'),(30,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
                    cv2.putText(imgInput,('Following Black Line'),(230,50), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,0),1,cv2.LINE_AA)

                cv2.putText(imgInput,(self.CVCommand),(30,90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1,cv2.LINE_AA)
                cv2.putText(imgInput,(self.CVCommand),(230,90), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,0,0),1,cv2.LINE_AA)

                cv2.line(imgInput,(self.left_Pos1,(linePos_1+30)),(self.left_Pos1,(linePos_1-30)),(255,255,255),1)
                cv2.line(imgInput,((self.left_Pos1+1),(linePos_1+30)),((self.left_Pos1+1),(linePos_1-30)),(0,0,0),1)

                cv2.line(imgInput,(self.right_Pos1,(linePos_1+30)),(self.right_Pos1,(linePos_1-30)),(255,255,255),1)
                cv2.line(imgInput,((self.right_Pos1-1),(linePos_1+30)),((self.right_Pos1-1),(linePos_1-30)),(0,0,0),1)

                cv2.line(imgInput,(0,linePos_1),(640,linePos_1),(255,255,255),1)
                cv2.line(imgInput,(0,linePos_1+1),(640,linePos_1+1),(0,0,0),1)

                cv2.line(imgInput,(320-findLineError,0),(320-findLineError,480),(255,255,255),1)
                cv2.line(imgInput,(320+findLineError,0),(320+findLineError,480),(255,255,255),1)

                cv2.line(imgInput,(320-findLineError+1,0),(320-findLineError+1,480),(0,0,0),1)
                cv2.line(imgInput,(320+findLineError-1,0),(320+findLineError-1,480),(0,0,0),1)

                cv2.line(imgInput,(self.left_Pos2,(linePos_2+30)),(self.left_Pos2,(linePos_2-30)),(255,255,255),1)
                cv2.line(imgInput,(self.right_Pos2,(linePos_2+30)),(self.right_Pos2,(linePos_2-30)),(255,255,255),1)
                cv2.line(imgInput,(0,linePos_2),(640,linePos_2),(255,255,255),1)

                cv2.line(imgInput,(self.left_Pos2+1,(linePos_2+30)),(self.left_Pos2+1,(linePos_2-30)),(0,0,0),1)
                cv2.line(imgInput,(self.right_Pos2-1,(linePos_2+30)),(self.right_Pos2-1,(linePos_2-30)),(0,0,0),1)
                cv2.line(imgInput,(0,linePos_2+1),(640,linePos_2+1),(0,0,0),1)

                cv2.line(imgInput,((self.center-20),int((linePos_1+linePos_2)/2)),((self.center+20),int((linePos_1+linePos_2)/2)),(0,0,0),1)
                cv2.line(imgInput,((self.center),int((linePos_1+linePos_2)/2+20)),((self.center),int((linePos_1+linePos_2)/2-20)),(0,0,0),1)

                cv2.line(imgInput,((self.center-20),int((linePos_1+linePos_2)/2+1)),((self.center+20),int((linePos_1+linePos_2)/2+1)),(255,255,255),1)
                cv2.line(imgInput,((self.center+1),int((linePos_1+linePos_2)/2+20)),((self.center+1),int((linePos_1+linePos_2)/2-20)),(255,255,255),1)
            except:
                pass

        elif self.CVMode == 'watchDog':
            if self.drawing:
                cv2.putText(imgInput,'Motion Detected',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                robot.buzzerCtrl(1, 0)
                robot.lightCtrl('red', 0)
                cv2.rectangle(imgInput, (self.mov_x, self.mov_y), (self.mov_x + self.mov_w, self.mov_y + self.mov_h), (128, 255, 0), 1)
            else:
                cv2.putText(imgInput,'Motion Detecting',(40,60), CVThread.font, 0.5,(255,255,255),1,cv2.LINE_AA)
                robot.buzzerCtrl(0, 0)
                robot.lightCtrl('blue', 0)

        return imgInput


    def watchDog(self, imgInput):
        timestamp = datetime.datetime.now()
        gray = cv2.cvtColor(imgInput, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (21, 21), 0)

        if self.avg is None:
            print("[INFO] starting background model...")
            self.avg = gray.copy().astype("float")
            return 'background model'

        cv2.accumulateWeighted(gray, self.avg, 0.5)
        self.frameDelta = cv2.absdiff(gray, cv2.convertScaleAbs(self.avg))

        # threshold the delta image, dilate the thresholded image to fill
        # in holes, then find contours on thresholded image
        self.thresh = cv2.threshold(self.frameDelta, 5, 255,
            cv2.THRESH_BINARY)[1]
        self.thresh = cv2.dilate(self.thresh, None, iterations=2)
        self.cnts = cv2.findContours(self.thresh.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)
        self.cnts = imutils.grab_contours(self.cnts)
        # print('x')
        # loop over the contours
        for c in self.cnts:
            # if the contour is too small, ignore it
            if cv2.contourArea(c) < 2000:
                continue
     
            # compute the bounding box for the contour, draw it on the frame,
            # and update the text
            (self.mov_x, self.mov_y, self.mov_w, self.mov_h) = cv2.boundingRect(c)
            self.drawing = 1
            
            self.motionCounter += 1

            self.lastMovtionCaptured = timestamp

        if (timestamp - self.lastMovtionCaptured).seconds >= 0.5:
            self.drawing = 0
            robot.buzzerCtrl(0, 0)

        self.pause()


    def findLineTest(self, posInput, setCenter):#2
        if not posInput:
            robot.robotCtrl.moveStart(speedMove, 'no', 'no')
            return

        if posInput > (setCenter + findLineError):
            self.CVCommand = 'Turning Right'

        elif posInput < (setCenter - findLineError):
            self.CVCommand = 'Turning Left'

        else:
            self.CVCommand = 'Forward'


    def findLineCtrl(self, posInput, setCenter):#2
        if not posInput:
            robot.robotCtrl.moveStart(speedMove, 'no', 'no')
            return

        if posInput > (setCenter + findLineError):
            #turnRight
            robot.right()
            self.CVCommand = 'Turning Right'
            print('Turning Right')

        elif posInput < (setCenter - findLineError):
            #turnLeft
            robot.left()
            self.CVCommand = 'Turning Left'
            print('Turning Left')

        else:
            #forward
            robot.forward()
            self.CVCommand = 'Forward'
            print('Forward')


    def findlineCV(self, frame_image):
        frame_findline = cv2.cvtColor(frame_image, cv2.COLOR_BGR2GRAY)
        retval, frame_findline =  cv2.threshold(frame_findline, 0, 255, cv2.THRESH_OTSU)
        frame_findline = cv2.erode(frame_findline, None, iterations=6)
        colorPos_1 = frame_findline[linePos_1]
        colorPos_2 = frame_findline[linePos_2]
        try:
            lineColorCount_Pos1 = np.sum(colorPos_1 == lineColorSet)
            lineColorCount_Pos2 = np.sum(colorPos_2 == lineColorSet)

            lineIndex_Pos1 = np.where(colorPos_1 == lineColorSet)
            lineIndex_Pos2 = np.where(colorPos_2 == lineColorSet)

            if lineColorCount_Pos1 == 0:
                lineColorCount_Pos1 = 1
            if lineColorCount_Pos2 == 0:
                lineColorCount_Pos2 = 1

            self.left_Pos1 = lineIndex_Pos1[0][lineColorCount_Pos1-1]
            self.right_Pos1 = lineIndex_Pos1[0][0]
            self.center_Pos1 = int((self.left_Pos1+self.right_Pos1)/2)

            self.left_Pos2 = lineIndex_Pos2[0][lineColorCount_Pos2-1]
            self.right_Pos2 = lineIndex_Pos2[0][0]
            self.center_Pos2 = int((self.left_Pos2+self.right_Pos2)/2)

            self.center = int((self.center_Pos1+self.center_Pos2)/2)
        except:
            center = None
            pass

        if Camera.CVMode == 'run':
            self.findLineCtrl(self.center, 320)
        else:
            self.findLineTest(self.center, 320)
        self.pause()


    def findColor(self, frame_image):
        hsv = cv2.cvtColor(frame_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, colorLower, colorUpper)#1
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            X_LOCK = 0
            Y_LOCK = 0
            self.findColorDetection = 1
            c = max(cnts, key=cv2.contourArea)
            ((self.box_x, self.box_y), self.radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            X = int(self.box_x)
            Y = int(self.box_y)
            error_Y = abs(240 - Y)
            error_X = abs(320 - X)

            if Y < 240 - CVThread.tor:
                # error_Y*CVThread.aspd
                robot.lookUp()
            elif Y > 240 + CVThread.tor:
                robot.lookDown()
            else:
                Y_LOCK = 1

            if X < 320 - CVThread.tor:
                robot.lookLeft()
            elif X > 320 + CVThread.tor:
                robot.lookRight()
            else:
                X_LOCK = 1

            if X_LOCK == 1 and Y_LOCK == 1:
                robot.buzzerCtrl(1, 0)
                robot.lightCtrl('red', 0)
            else:
                robot.buzzerCtrl(0, 0)
                robot.lightCtrl('blue', 0)

        else:
            self.findColorDetection = 0
        self.pause()


    def faceDetectCV(self, frame_image):
        grayGen = cv2.cvtColor(frame_image, cv2.COLOR_BGR2GRAY)
        self.faces = faceCascade.detectMultiScale(
                grayGen,     
                scaleFactor=1.2,
                minNeighbors=5,     
                minSize=(20, 20)
            )
        if len(self.faces):
            robot.lightCtrl('red', 0)
            robot.buzzerCtrl(1, 0)
        else:
            robot.lightCtrl('blue', 0)
            robot.buzzerCtrl(0, 0)
        self.pause()


    def pause(self):
        self.__flag.clear()

    def resume(self):
        self.__flag.set()

    def run(self):
        while 1:
            if self.CVMode == 'none':
                robot.buzzerCtrl(0, 0)

            self.__flag.wait()
            if self.CVMode == 'none':
                robot.stopLR()
                robot.stopFB()
                robot.buzzerCtrl(0, 0)
                robot.lightCtrl('blue', 0)
                self.pause()
                robot.buzzerCtrl(0, 0)
                continue

            elif self.CVMode == 'findColor':
                self.CVThreading = 1
                self.findColor(self.imgCV)
                self.CVThreading = 0

            elif self.CVMode == 'findlineCV':
                self.CVThreading = 1
                self.findlineCV(self.imgCV)
                self.CVThreading = 0

            elif self.CVMode == 'watchDog':
                self.CVThreading = 1
                self.watchDog(self.imgCV)
                self.CVThreading = 0

            elif self.CVMode == 'faceDetection':
                self.CVThreading = 1
                self.faceDetectCV(self.imgCV)
                self.CVThreading = 0


class Camera(BaseCamera):
    video_source = 0
    modeSelect = 'none'
    # modeSelect = 'findlineCV'
    # modeSelect = 'findColor'
    # modeSelect = 'watchDog'
    # add # modeSelect = 'faceDetection'

    CVMode = 'run'
    # CVMode = 'no'

    def __init__(self):
        if os.environ.get('OPENCV_CAMERA_SOURCE'):
            Camera.set_video_source(int(os.environ['OPENCV_CAMERA_SOURCE']))
        super(Camera, self).__init__()

    def robotStop(self):
        robot.robotCtrl.moveStart(speedMove, 'no', 'no')
        time.sleep(0.1)
        robot.robotCtrl.moveStart(speedMove, 'no', 'no')

    def colorFindSet(self, invarH, invarS, invarV):
        global colorUpper, colorLower
        HUE_1 = invarH+15
        HUE_2 = invarH-15
        if HUE_1>180:HUE_1=180
        if HUE_2<0:HUE_2=0

        SAT_1 = invarS+150
        SAT_2 = invarS-150
        if SAT_1>255:SAT_1=255
        if SAT_2<0:SAT_2=0

        VAL_1 = invarV+150
        VAL_2 = invarV-150
        if VAL_1>255:VAL_1=255
        if VAL_2<0:VAL_2=0

        colorUpper = np.array([HUE_1, SAT_1, VAL_1])
        colorLower = np.array([HUE_2, SAT_2, VAL_2])
        print('HSV_1:%d %d %d'%(HUE_1, SAT_1, VAL_1))
        print('HSV_2:%d %d %d'%(HUE_2, SAT_2, VAL_2))
        print(colorUpper)
        print(colorLower)

    def modeSet(self, invar):
        Camera.modeSelect = invar

    def upperIP(self, invar):
        global upperGlobalIP
        upperGlobalIP = invar

    def CVRunSet(self, invar):
        global CVRun
        CVRun = invar

    def linePosSet_1(self, invar):
        global linePos_1
        linePos_1 = invar

    def linePosSet_2(self, invar):
        global linePos_2
        linePos_2 = invar

    def colorSet(self, invar):
        global lineColorSet
        lineColorSet = invar

    def randerSet(self, invar):
        global frameRender
        frameRender = invar

    def errorSet(self, invar):
        global findLineError
        findLineError = invar

    @staticmethod
    def set_video_source(source):
        Camera.video_source = source

    @staticmethod
    def frames():
        camera = cv2.VideoCapture(Camera.video_source)
        camera.set(3, 640)
        camera.set(4, 480)
        if not camera.isOpened():
            raise RuntimeError('Could not start camera.')

        cvt = CVThread()
        cvt.start()

        while True:
            # read current frame
            _, img = camera.read()

            if Camera.modeSelect == 'none':
                cvt.pause()
                robot.buzzerCtrl(0, 0)
            else:
                if cvt.CVThreading:
                    pass
                else:
                    cvt.mode(Camera.modeSelect, img)
                    cvt.resume()
                try:
                    img = cvt.elementDraw(img)
                except:
                    pass

            # encode as a jpeg image and return it
            try:
                yield cv2.imencode('.jpg', img)[1].tobytes()
            except:
                pass


def commandAct(act, inputA):
    global speedMove
    if act == 'forward':
        robot.forward(speedMove)
    elif act == 'backward':
        robot.backward(speedMove)
    elif act == 'left':
        robot.left(speedMove)
    elif act == 'right':
        robot.right(speedMove)
    elif act == 'DS':
        robot.stopFB()
    elif act == 'TS':
        robot.stopLR()

    elif 'wsB' in act:
        speedMove = int(act.split()[1])
        if(speedMove > 1 and speedMove <= 100):
            robot.speedSet(speedMove)

    elif act == 'up':
        robot.lookUp()
    elif act == 'down':
        robot.lookDown()
    elif act == 'UDstop':
        robot.lookStopUD()
    elif act == 'lookleft':
        robot.lookLeft()
    elif act == 'lookright':
        robot.lookRight()
    elif act == 'LRstop':
        robot.lookStopLR()

    elif act == 'jump':
        robot.jump()
    elif act == 'handshake':
        robot.handShake()
    elif act == 'steady':
        robot.steadyMode()
    elif act == 'steadyOff':
        robot.steadyMode()

    # openCV ctrl.
    elif act == 'faceDetection':
        Camera.modeSelect = 'faceDetection'
    elif act == 'faceDetectionOff':
        Camera.modeSelect = 'none'
        robot.buzzerCtrl(0, 0)
    elif 'trackLine' == act:
        Camera.modeSelect = 'findlineCV'
        Camera.CVMode = 'run'
    elif 'trackLineOff' == act:
        Camera.modeSelect = 'none'
        time.sleep(0.05)
        robot.stopLR()
        time.sleep(0.05)
        robot.stopFB()
        robot.buzzerCtrl(0, 0)