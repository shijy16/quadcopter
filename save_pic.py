import vrep
import time
import ctypes
import numpy as np
import time
import pdb
import array
from PIL import Image
import matplotlib.pyplot as plt

def save_pic(zed_vision1):
    print ('Program started')
    vrep.simxFinish(-1) # just in case, close all opened connections
    clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
    # clientID=vrep.simxStart('127.0.0.1',20000,True,True,5000,5) # Connect to V-REP


    if clientID!=-1:
        print ('Connected to remote API server')

        time.sleep(2)

        # Now retrieve streaming data (i.e. in a non-blocking fashion):
        startTime=time.time()

        
        err, camera1 = vrep.simxGetObjectHandle(clientID, zed_vision1,
                                                    vrep.simx_opmode_blocking )
        print('err camera1:', err)

        print('c1_handle', camera1)
        
        #save picture from zed
        print("Testing vision sensor!!!")
        # pdb.set_trace()

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
                plt.imshow(img)
                im=Image.fromarray(img)
                new_image = im.transpose(Image.ROTATE_180)
                new_image.show()
                now_time = time.time()
                new_image.save('./Data/'+str(now_time)+'.jpg')
                break

        # Now close the connection to V-REP:
        vrep.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
    print ('Program ended')

if __name__ == '__main__':
    str1 = "zed_vision1"
    save_pic(str1)