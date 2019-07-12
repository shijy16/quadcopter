import vrep
import ctypes
import numpy as np
import time
import math
import gearControl
import util
import controller
import _thread
PI = 3.1415926

class PlaneCotroller:
    #=============================================================#
    #    following functions only used in upper class functions   #
    #=============================================================#
    def __init__(self,cid):
        self.clientId = cid
        err, self.copter = vrep.simxGetObjectHandle(self.clientId, "Quadricopter_base",
                                                vrep.simx_opmode_oneshot_wait )
        err, self.target = vrep.simxGetObjectHandle(self.clientId, "Quadricopter_target",
                                                vrep.simx_opmode_oneshot_wait )
        ret, self.gearHandle1 = vrep.simxGetObjectHandle(self.clientId, 'Gear_joint1',
                                                vrep.simx_opmode_oneshot_wait)
        ret, self.gearHandle2 = vrep.simxGetObjectHandle(self.clientId, 'Gear_joint2',
                                                vrep.simx_opmode_oneshot_wait)
        self.plane_pos = self.get_object_pos(self.copter)
        # ret, gear_pos1 = vrep.simxGetJointPosition(self.clientId, self.gearHandle1, vrep.simx_opmode_streaming)
        # ret, gear_pos2 = vrep.simxGetJointPosition(self.clientId, self.gearHandle2, vrep.simx_opmode_streaming)
        # print(ret, gear_pos1, gear_pos2)
        self.vrep_mode = vrep.simx_opmode_oneshot
        while(self.check_target_pos()):
            None

    def send_motor_commands(self,values ):
        # Limit motors by max and min values
        motor_values = np.zeros(4)
        for i in range(4):
            motor_values[i] = values[i]
        packedData=vrep.simxPackFloats(motor_values.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        err = vrep.simxSetStringSignal(self.clientId, "rotorTargetVelocities",
                                        raw_bytes,
                                        self.vrep_mode)
    
    def send_power_commands(self,power ):
        # Limit motors by max and min values
        motor_values = np.zeros(1)
        motor_values[0] = power
        packedData=vrep.simxPackFloats(motor_values.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        err = vrep.simxSetStringSignal(self.clientId, "rotorPower",
                                        raw_bytes,
                                        self.vrep_mode)

    def up_gear(self):
        gearControl.send_gear_commands(self.clientId, -60.0, 60.0, self.gearHandle1, self.gearHandle2)

    def down_gear(self):
        gearControl.send_gear_commands(self.clientId, 0.0, 0.0, self.gearHandle1, self.gearHandle2)


    def set_object_pos(self,pos,obj):
        vrep.simxSetObjectPosition(self.clientId,obj,-1,pos,self.vrep_mode)
    
    def get_object_pos(self,obj):
        err,pos = vrep.simxGetObjectPosition(self.clientId,obj,-1,vrep.simx_opmode_blocking)
        return pos
    
    def get_current_pos(self):
        pos = self.get_object_pos(self.copter)
        self.plane_pos = pos
        return pos
    
    # def update_pos(self):
    #     self.get_current_pos()
    #     self.get_target_pos()

    def get_object_orientation(self,obj):
        err,ori = vrep.simxGetObjectOrientation(self.clientId,obj,-1,vrep.simx_opmode_blocking)
        return ori
    
    def set_target_pos(self,pos):
        print('set',pos)
        self.target_pos = pos
        vrep.simxSetObjectPosition(self.clientId,self.target,-1,pos,self.vrep_mode)
    
    def get_target_pos(self):
        err,pos = vrep.simxGetObjectPosition(self.clientId,self.target,-1,vrep.simx_opmode_blocking)
        self.target_pos = pos
        return pos
    
    def set_target_orientation(self,orientation):
        vrep.simxSetObjectOrientation(self.clientId,self.target,-1,orientation,self.vrep_mode)
    
    def get_target_orientation(self):
        err,orientation = vrep.simxGetObjectOrientation(self.clientId,self.target,-1,vrep.simx_opmode_blocking)
        return orientation
    
    def check_target_pos(self):
        self.target_pos = self.get_object_pos(self.target)
        if self.target_pos[0] == 0 and self.target_pos[1] == 0 and self.target_pos[2] == 0:
            return True
        # print(self.target_pos)
        return False
    
    def check_target_orientation(self):
        self.target_orientation = self.get_target_orientation()
        if self.target_orientation[0] == 0 and self.target_orientation[1] == 0 and self.target_orientation[2] == 0:
            return True
        return False

    def up(self,h=0.05):
        if(self.check_target_pos()):
            return
        self.target_pos[2] += h
        self.move_to(self.target_pos)
    
    def down(self,h=0.05):
        if(self.check_target_pos()):
            return
        self.target_pos[2] -= h
        self.move_to(self.target_pos)

    
    def get_delta(self,l1,l2):
        delta = [ abs(l1[0]-l2[0]),abs(l1[1]-l2[1]),abs(l1[2]-l2[2]) ]
        if delta[0] > delta[1]:
            if delta[0] > delta[2]:
                return delta[0]
            else:
                return delta[2]
        elif delta[1] > delta[2]:
            return delta[1]
        else:
            return delta[2]
    
    def rotate_to(self,angle):
        dir = 1
        target_ori = self.get_target_orientation()[2]
        if(angle - target_ori < 0 and abs(angle - target_ori) < PI):
            dir = -1
        if(angle - target_ori > 0 and abs(angle - target_ori) > PI):
            dir = -1
        while(abs(angle - target_ori) > 0.0001):
            if(abs(target_ori - angle) < PI/9.0):
                target_ori = angle
            else:
                target_ori = target_ori + PI/9.0*dir
            cur_ori =self.get_object_orientation(self.copter)[2]
            while(abs(cur_ori - target_ori) > 0.01):
                print(target_ori)
                self.set_target_orientation([0,0,target_ori])
                cur_ori =self.get_object_orientation(self.copter)[2]
                time.sleep(0.1)


    #=============================================================#
    #        use following functions to controll the plane        #
    #=============================================================#

    def get_camera_pic(self,camera_pos):
        img = None
        if camera_pos == 1:
            img = util.save_pic('zed_vision1',self.clientId)
            while(img is None):
                img = util.save_pic('zed_vision1',self.clientId)
        else:
            img = util.save_pic('zed_vision0',self.clientId)
            while(img is None):
                img = util.save_pic('zed_vision0',self.clientId)
        return img

    #hard means this move requires high currency
    def to_height(self,h,max_v=0.2,t=-1):
        self.move_to([self.plane_pos[0],self.plane_pos[1],h],max_v=max_v,t=t)
        self.plane_pos[2] = h

    def move_horizontally(self,x,y,hard=True):
        self.move_to([x,y,self.plane_pos[2]],hard)

    def stable_move(self,delta_x,delta_y):
        self.set_target_pos([self.target_pos[0] + delta_x,self.target_pos[1] + delta_y,self.target_pos[2]])
        self.plane_pos[0] += delta_x
        self.plane_pos[1] += delta_y
        time.sleep(5)
        v1 = [1,1,1]
        while(self.get_delta(v1,[0,0,0]) > 0.02):
            time.sleep(0.5)
            err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)

    def move_to(self,dest,hard = True,max_v = 0.02,t=-1):
        delta = [0,0,0]
        delta[0] = dest[0] - self.plane_pos[0]
        delta[1] = dest[1] - self.plane_pos[1]
        delta[2] = dest[2] - self.plane_pos[2]
        print(self.plane_pos,delta,dest,self.target_pos)
        self.set_target_pos([self.target_pos[0] + delta[0],self.target_pos[1] + delta[1],self.target_pos[2] + delta[2]])
        if(t != -1):
            print(t)
            time.sleep(t)
            return
        time.sleep(5)
        v1 = [1,1,1]
        if(not hard):
            max_v = 0.05
        while(self.get_delta(v1,[0,0,0]) > max_v):
            time.sleep(0.5)
            err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)
        self.plane_pos = dest
        # time.sleep(3)
        # v1 = [1,1,1]
        # if hard:
        #     while(self.get_delta(v1,[0,0,0]) > 0.01):
        #         time.sleep(0.5)
        #         err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)
        #     print('correcting current pos...')
        #     cur = self.get_object_pos(self.copter)
        #     target_pos = self.target_pos
        #     delta = [0,0,0]
        #     delta[0] = dest[0] - cur[0]
        #     delta[1] = dest[1] - cur[1]
        #     delta[2] = dest[2] - cur[2]
        #     print('delta',delta)
        #     target_pos[0] += delta[0]
        #     target_pos[1] += delta[1]
        #     target_pos[2] += delta[2]
        #     self.set_target_pos(target_pos)
        #     time.sleep(5)
        #     err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)
        #     while(self.get_delta(v1,[0,0,0]) > 0.01):
        #         time.sleep(0.5)
        #         err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)
        #     # print(self.get_current_pos())
        #     print("arrive")
        #     # time.sleep(10)
        # else:
        #     while(self.get_delta(v1,[0,0,0]) > 0.05):
        #         time.sleep(0.5)
        #         err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)
        #         self.plane_pos[0] -= 0.19

        # dir = [dest[0] - self.target_pos[0],dest[1] - self.target_pos[1],dest[2] - self.target_pos[2]]
        # len = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]
        # len = math.sqrt(len)
        # if(len < 1e-2):
        #     return
        # dir[0] /= (len*100.0)
        # dir[1] /= (len*100.0)
        # dir[2] /= (len*100.0)
        # while(self.get_delta(self.target_pos,dest) > 0.01):
        #     # err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)
        #     # len = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]
        #     # if(math.sqrt(v1[0]*v1[0] + v1[2]*v1[2] + v1[1]*v1[1]) > math.sqrt(len)*195):
        #     #     dir[0]*=2
        #     #     dir[1]*=2
        #     #     dir[2]*=2
        #     # else:
        #     #     dir[0]/=2
        #     #     dir[1]/=2
        #     #     dir[2]/=2
        #     if(self.get_delta(self.target_pos,dest) > 0.05):
        #         self.target_pos[0] += dir[0]
        #         self.target_pos[1] += dir[1]
        #         self.target_pos[2] += dir[2]
        #     else:
        #         self.target_pos = dest
        #     self.set_target_pos(self.target_pos)
        #     # time.sleep(0.1)
        #     vrep.simxGetPingTime(self.clientId)
        #     vrep.simxSynchronousTrigger(self.clientId)
            
    def loose_jacohand(self):
        motor_values = np.zeros(1)
        motor_values[0] = -1
        packedData=vrep.simxPackFloats(motor_values.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        err = vrep.simxSetStringSignal(self.clientId, "jacohand",
                                        raw_bytes,
                                        self.vrep_mode)

    def grap_jacohand(self):
        motor_values = np.zeros(1)
        motor_values[0] = 1
        packedData=vrep.simxPackFloats(motor_values.flatten())
        raw_bytes = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        err = vrep.simxSetStringSignal(self.clientId, "jacohand",
                                        raw_bytes,
                                        self.vrep_mode)       

    def take_off(self):
        self.send_power_commands(0)
        # self.up_gear()

    def landing(self):
        # self.down_gear()
        time.sleep(1)
        self.send_power_commands(-5)
        time.sleep(3)
        self.send_power_commands(-9)  
    

    #mission 2
    def get_target_platform_pos(self):
        ret, _, target_platform_pos, _, _ = vrep.simxCallScriptFunction(self.clientId, 'util_funcs', vrep.sim_scripttype_customizationscript, 
                                        'my_get_target_platform_pos', [], [], [], bytearray(), vrep.simx_opmode_oneshot_wait)
        return target_platform_pos

    def get_target_info(self):
        #try to be stable
        err,ori = vrep.simxGetObjectOrientation(self.clientId,self.copter,-1,self.vrep_mode)
        while(abs(ori[0] > 5.0/180.0*PI) or abs(ori[1] > 10.0/180.0*PI) or abs(ori[2] > 5.0/180.0*PI)):
            err,ori = vrep.simxGetObjectOrientation(self.clientId,self.copter,-1,self.vrep_mode)
        img1 = self.get_camera_pic(0)
        #try to be stable
        err,ori = vrep.simxGetObjectOrientation(self.clientId,self.copter,-1,self.vrep_mode)
        while(abs(ori[0] > 5.0/180.0*PI) or abs(ori[1] > 10.0/180.0*PI) or abs(ori[2] > 5.0/180.0*PI)):
            err,ori = vrep.simxGetObjectOrientation(self.clientId,self.copter,-1,self.vrep_mode)
        img2 = self.get_camera_pic(1)
        center1,size1 = util.find_target(img1)
        center2,size2 = util.find_target(img2)

        center = [0,0]
        size = [0,0]
        center[0] = (center1[0] + center2[0])/2.0
        center[1] = (center1[1] + center2[1])/2.0
        size[0] = (size1[0] + size2[0])/2.0
        size[1] = (size1[1] + size2[1])/2.0
        err = 0
        if(size1[0] == 0):
            err += 1
        if(size2[0] == 0):
            err += 1
        return err,center,size
    
    def grap_target(self):
        self.loose_jacohand()
        print('grap the target...')
        #goto platform
        self.to_height(2)

        platform_pos = self.get_target_platform_pos()
        self.move_horizontally(platform_pos[0],platform_pos[1])
        print("arrive got position",platform_pos)
        self.rotate_to(0)
        err,center,size = self.get_target_info()
        while(err != 0):
            platform_pos = self.get_target_platform_pos()
            self.move_horizontally(platform_pos[0],platform_pos[1])
            err,center,size = self.get_target_info()
        print('target found')
        

        #---------------calculate target pos roughly-------------------
        print("height beyond target:",85/size[0],"target size:",size,"target center:",center)
        delta_pos = [0,0]
        delta_pos[1] = center[0] - 644
        delta_pos[0] = center[1] - 389
        # print("delta",delta_pos,"delta indeed",[platform_pos[0] - 7.225,platform_pos[1]+10.425])
        delta_pos[0] /= 450
        delta_pos[1] /= 450
        print("move to directly above target",delta_pos)
        self.move_horizontally(self.plane_pos[0] - delta_pos[0],self.plane_pos[1] + delta_pos[1])

        print('move down a little and calculate position')
        self.to_height(1.5)
        #---------------calculate target pos-------------------
        err,center,size = self.get_target_info()
        print("height beyond target:",85/size[0],"target size:",size,"target center:",center)
        delta_pos = [0,0]
        delta_pos[1] = center[0] - 640
        delta_pos[0] = center[1] - 410
        # print("delta",delta_pos,"delta indeed",[platform_pos[0] - 7.225,platform_pos[1]+10.425])
        delta_pos[0] /= 630
        delta_pos[1] /= 630
        print("move to directly above target",delta_pos)
        
        target_x,target_y = self.plane_pos[0] - delta_pos[0],self.plane_pos[1] + delta_pos[1]
        self.move_horizontally(self.plane_pos[0] - delta_pos[0],self.plane_pos[1] + delta_pos[1])

        time.sleep(5)
        print("move down")
        self.to_height(0.75)
        err,center,size = self.get_target_info()
        delta_pos = [0,0]
        delta_pos[0] = center[0] - 640
        delta_pos[1] = center[1] - 500
        while(abs(delta_pos[1]) > 30 or abs(delta_pos[0]) > 30 or err != 0):
            print(delta_pos,err,center)
            if(err != 0):
                self.stable_move(-0.03,0)
            else:
                x = 0
                y = 0
                if(delta_pos[1] > 30):
                    x = -0.03
                elif(delta_pos[1] < -30):
                    x = 0.03
                if(delta_pos[0] > 30):
                    y = 0.03
                elif(delta_pos[0] < -30):
                    y = -0.03
                self.stable_move(x,y)
            err,center,size = self.get_target_info()
            delta_pos[0] = center[0] - 640
            delta_pos[1] = center[1] - 500
        self.to_height(0.45,t=7)
        self.grap_jacohand()
        time.sleep(2.5)
        self.to_height(1.5)
        time.sleep(10)
        self.plane_pos = self.get_object_pos(self.copter)


class MainController:
    def __init__(self, *args, **kwargs):
        print ('Program started')
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientId=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP, set a very large time-out for blocking commands
    
    def pdThread(self):
        while(True):
            self.pdController.control_step()
    #=============================================================#
    #                 don't change this function                  #
    #=============================================================#
    def startSimulation(self):
        if self.clientId!=-1:
            #+++++++++++++++++++++++++++++++++++++++++++++
            step = 0.005
            vrep.simxSetFloatingParameter(self.clientId, vrep.sim_floatparam_simulation_time_step, step, vrep.simx_opmode_oneshot)
            vrep.simxSynchronous(self.clientId, True)
            vrep.simxStartSimulation(self.clientId,vrep.simx_opmode_oneshot)
            #init the controller
            planeController = PlaneCotroller(self.clientId)
            planeController.take_off()
            self.pdController = controller.PID(cid=self.clientId)

            #create a thread to controll the move of quadcopter
            _thread.start_new_thread(self.pdThread,())
            print("thread created")
            #to be stable
            planeController.loose_jacohand()
            planeController.move_to(planeController.get_object_pos(planeController.copter),False)
            time.sleep(10)
            planeController.plane_pos = planeController.get_object_pos(planeController.copter)
            time.sleep(10)
            self.run_simulation(planeController)
        else:
            print ('Failed connecting to remote API server')
        print ('Simulation ended')
        vrep.simxStopSimulation(self.clientId,vrep.simx_opmode_oneshot)
    
    #=============================================================#
    #                 simulation runs here                        #
    #=============================================================#
    def run_simulation(self,planeController):
        # while(True):
        #     None
        print("run simulation")
        # planeController.move_to([7.225,-10.425,0.75])
        # # 640 389 32 22
        # print(planeController.get_target_info())
        # planeController.move_to([7.225,-10.425,2])
        # #644 389 46 26
        # print(planeController.get_target_info())

        # planeController.move_to([7.225,-10.425,1.5])
        #640 410 73 41

        # print(planeController.get_target_info())
        planeController.grap_target()
        planeController.landing()
        time.sleep(100)



mainController = MainController()
mainController.startSimulation()