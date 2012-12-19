import numpy as np
import cv2
import cv2.cv as cv
from video import create_capture
from common import clock, draw_str
import commands
import sys

'''
Functions to setup the webcam and return co-ordinates of face
export PYTHONPATH=<path to samples/python2>
^^^ uses video and common from this directory
'''

OpenCVInstallDir = "/home/prathmesh/Work/OpenCV-2.4.0/"

if commands.getoutput("cd " + OpenCVInstallDir) : 
    print "Change OpenCVInstallDir to your installation directory in LookingAt.py"
    sys.exit(-1)

def detect(img, cascade):
    rects = cascade.detectMultiScale(img, scaleFactor=1.3, minNeighbors=4, minSize=(30, 30), flags = cv.CV_HAAR_SCALE_IMAGE)
    if len(rects) == 0:
        return []
    rects[:,2:] += rects[:,:2]
    return rects

def faceDetect(loopCount) : 
    import sys, getopt
    
    args, video_src = getopt.getopt(sys.argv[1:], '', ['cascade=', 'nested-cascade='])
    try: video_src = video_src[0]
    except: video_src = 0
    args = dict(args)
    cascade_fn = args.get('--cascade', OpenCVInstallDir + "data/haarcascades/haarcascade_frontalface_alt.xml")
    cascade = cv2.CascadeClassifier(cascade_fn)
    cam = create_capture(video_src, fallback='synth:bg=../cpp/lena.jpg:noise=0.05')
    idx = 0

    if loopCount==0 : 
        loopCount = 1
        infinteLoop = True
    else : 
        infinteLoop = False

    while idx<loopCount:
        ret, img = cam.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        
        t = clock()
        rects = detect(gray, cascade)
        vis = img.copy()
        dt = clock() - t

        draw_str(vis, (20, 20), 'time: %.1f ms' % (dt*1000))
        if rects == [] : 
            draw_str(vis, (20, 40), 'We are having trouble seeing you, move around just a bit')
#            draw_str(vis,(20,450), 'Look Here')
        else : 
            if infinteLoop : 
                idx = 0
                print rects
            else : 
                idx = idx + 1
                try : 
                    rectsum = rectsum + rects
                except : 
                    rectsum = rects
                    #first time assignment

#       cv2.imshow('facetracker', vis)


        if 0xFF & cv2.waitKey(5) == 27:
            break
    cv2.destroyAllWindows() 			
    return rectsum/idx

abs_top_left = [400, 50]
abs_top_rigt = [30, 50]
abs_bot_left = [400, 200]
abs_bot_rigt = [30, 200]

abs_lt = [30, 50]
abs_rb = [400, 200]

import numpy

def userCalibrate() : 
    raw_input("Look at the top left corner of the screen \n Press enter when ready")
    retArray = faceDetect(10)
    print numpy.array(retArray)[0].tolist()
    raw_input("Look at the top right corner of the screen \n Press enter when ready")
    retArray = faceDetect(10)
    print numpy.array(retArray)[0].tolist()
    raw_input("Look at the bottom left corner of the screen \n Press enter when ready")
    retArray = faceDetect(10)
    print numpy.array(retArray)[0].tolist()
    raw_input("Look at the top right corner of the screen \n Press enter when ready")
    retArray = faceDetect(10)
    print numpy.array(retArray)[0].tolist()
    

if __name__ == "__main__" :
    userCalibrate()
