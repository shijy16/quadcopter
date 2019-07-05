import vrep
import time
import ctypes
import numpy as np
import time
import pdb
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
            print("geting...")
            res,resolution,image=vrep.simxGetVisionSensorImage(clientID,camera1,0,vrep.simx_opmode_buffer)
            if res==vrep.simx_return_ok:
                print(resolution,len(image))
                # print(image)
                img = np.array(image, dtype = np.uint8)
                img.resize([resolution[1],resolution[0],3])
                # plt.imshow(img)
                im=Image.fromarray(img)
                new_image = im.transpose(Image.ROTATE_180)
                # new_image.show()
                now_time = time.time()
                new_image.save('./pic/'+str(now_time)+'.jpg')
                return img
