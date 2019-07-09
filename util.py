import vrep
import time
import ctypes
import numpy as np
import time
import pdb
import cv2
import array
from PIL import Image
import matplotlib.pyplot as plt

def save_pic(vision_name,clientID):
    if clientID!=-1:
        err, camera1 = vrep.simxGetObjectHandle(clientID, vision_name,
                                                    vrep.simx_opmode_blocking )
        res,resolution,image=vrep.simxGetVisionSensorImage(clientID,camera1,0,vrep.simx_opmode_streaming)
        time.sleep(1)
        while (vrep.simxGetConnectionId(clientID)!=-1):
            print("getting pic...")
            res,resolution,image=vrep.simxGetVisionSensorImage(clientID,camera1,0,vrep.simx_opmode_buffer)
            if res==vrep.simx_return_ok:
                # print(resolution,len(image))
                # print(image)
                img = np.array(image, dtype = np.uint8)
                img.resize([resolution[1],resolution[0],3])
                # plt.imshow(img)
                im=Image.fromarray(img)
                new_image = im.transpose(Image.ROTATE_180)
                # new_image.show()
                # now_time = time.time()
                # new_image.save('./pic/'+str(now_time) + vision_name+'.jpg')
                return new_image

def drawRect(img, pt1, pt2, pt3, pt4, color, lineWidth):
    cv2.line(img, tuple(pt1), tuple(pt2), color, lineWidth)
    cv2.line(img, tuple(pt2), tuple(pt3), color, lineWidth)
    cv2.line(img, tuple(pt3), tuple(pt4), color, lineWidth)
    cv2.line(img, tuple(pt1), tuple(pt4), color, lineWidth)
    cv2.imshow("img",img)
    cv2.waitKey(0)

def find_target(img):
    img_hsv = cv2.cvtColor(np.asarray(img),cv2.COLOR_RGB2HSV)
    img = np.asarray(img)
    lower_red = np.array([0, 43, 46])
    upper_red = np.array([10, 255, 255])
    mask_red = cv2.inRange(img_hsv, lower_red, upper_red)
    cnts = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # lower_red = np.array([156, 43, 46])
    # upper_red = np.array([180, 255, 255])
    # mask_red = cv2.inRange(img_hsv, lower_red, upper_red)
    # cnts += cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(cnts) > 0: 
        c = max(cnts, key = cv2.contourArea)
        min_rect = cv2.minAreaRect(c)
        center = min_rect[0]
        w = min_rect[1][0]
        l = min_rect[1][1]
        if l < w:
            l,w = w,l
        return center,[l,w]
    else:
        return None
        # box = cv2.boxPoints(min_rect)
        # print(box)
        # drawRect(img,box[0],box[1],box[2],box[3],[0,0,0],1)
    
    # print('target',x,y,radius)
    # return x,y,radius

