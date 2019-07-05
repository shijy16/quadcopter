import vrep
import ctypes
import numpy as np
import time
import math
import gearControl

PI = 3.1415926

class PlaneCotroller:
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
        ret, gear_pos1 = vrep.simxGetJointPosition(self.clientId, self.gearHandle1, vrep.simx_opmode_streaming)
        ret, gear_pos2 = vrep.simxGetJointPosition(self.clientId, self.gearHandle2, vrep.simx_opmode_streaming)
        print(ret, gear_pos1, gear_pos2)
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

    def take_off(self):
        self.send_power_commands(9)
        self.up_gear()

    def landing(self):
        self.down_gear()
        time.sleep(1)
        self.send_power_commands(3)
        time.sleep(3)
        self.send_power_commands(0)

    def set_object_pos(self,pos,obj):
        vrep.simxSetObjectPosition(self.clientId,obj,-1,pos,self.vrep_mode)
    
    def get_object_pos(self,obj):
        err,pos = vrep.simxGetObjectPosition(self.clientId,obj,-1,self.vrep_mode)
        return pos
    
    def get_object_orientation(self,obj):
        err,ori = vrep.simxGetObjectOrientation(self.clientId,obj,-1,self.vrep_mode)
        return ori
    
    def set_target_pos(self,pos):
        vrep.simxSetObjectPosition(self.clientId,self.target,-1,pos,self.vrep_mode)
    
    def get_target_pos(self):
        err,pos = vrep.simxGetObjectPosition(self.clientId,self.target,-1,self.vrep_mode)
        return pos
    
    def set_target_orientation(self,orientation):
        vrep.simxSetObjectOrientation(self.clientId,self.target,-1,orientation,self.vrep_mode)
    
    def get_target_orientation(self):
        err,orientation = vrep.simxGetObjectOrientation(self.clientId,self.target,-1,self.vrep_mode)
        return orientation
    
    def check_target_pos(self):
        for i in range(0,10):
            self.target_pos = self.get_object_pos(self.copter)
        if self.target_pos[0] == 0 and self.target_pos[1] == 0 and self.target_pos[2] == 0:
            return True
        return False
    
    def check_target_orientation(self):
        self.target_orientation = self.get_target_orientation()
        if self.target_orientation[0] == 0 and self.target_orientation[1] == 0 and self.target_orientation[2] == 0:
            return True
        return False

    def up(self):
        if(self.check_target_pos()):
            return
        self.target_pos[2] += 0.05
        self.set_target_pos(self.target_pos)
    
    def down(self):
        if(self.check_target_pos()):
            return
        self.target_pos[2] -= 0.05
        self.set_target_pos(self.target_pos)
    
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

    def move_to(self,dest):
        print(self.target_pos)
        dir = [dest[0] - self.target_pos[0],dest[1] - self.target_pos[1],dest[2] - self.target_pos[2]]
        len = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]
        len = math.sqrt(len)
        dir[0] /= (len*200.0)
        dir[1] /= (len*200.0)
        dir[2] /= (len*200.0)
        while(self.get_delta(self.target_pos,dest) > 0.01):
            # err,v1,v2 = vrep.simxGetObjectVelocity(self.clientId,self.copter,self.vrep_mode)
            # len = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]
            # if(math.sqrt(v1[0]*v1[0] + v1[2]*v1[2] + v1[1]*v1[1]) > math.sqrt(len)*195):
            #     dir[0]*=2
            #     dir[1]*=2
            #     dir[2]*=2
            # else:
            #     dir[0]/=2
            #     dir[1]/=2
            #     dir[2]/=2
            if(self.get_delta(self.target_pos,dest) > 0.05):
                self.target_pos[0] += dir[0]
                self.target_pos[1] += dir[1]
                self.target_pos[2] += dir[2]
            else:
                self.target_pos = dest
            self.set_target_pos(self.target_pos)
            time.sleep(0.025)
            vrep.simxSynchronousTrigger(self.clientId)
            
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

class MainController:
    def __init__(self, *args, **kwargs):
        print ('Program started')
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientId=vrep.simxStart('127.0.0.1',19997,True,True,5000,5) # Connect to V-REP, set a very large time-out for blocking commands
    
    def startSimulation(self):
        if self.clientId!=-1:
            #+++++++++++++++++++++++++++++++++++++++++++++
            step = 0.005
            vrep.simxSetFloatingParameter(self.clientId, vrep.sim_floatparam_simulation_time_step, step, vrep.simx_opmode_oneshot)
            # vrep.simxSynchronous(self.clientId, True)
            vrep.simxStartSimulation(self.clientId,vrep.simx_opmode_oneshot)
            # time.sleep(2)
            vrep.simxSynchronousTrigger(self.clientId)
            # planeController = PlaneCotroller(self.clientId)
            # planeController.up_gear()
            # planeController.down_gear()
            self.run_simulation()
        else:
            print ('Failed connecting to remote API server')
        print ('Simulation ended')
        vrep.simxStopSimulation(self.clientId,vrep.simx_opmode_oneshot)
    
    def run_simulation(self):
        planeController = PlaneCotroller(self.clientId)
        planeController.take_off()
        # while(True):
        #     planeController.set_target_orientation([0,0,0])
        #     planeController.set_target_pos(planeController.get_object_pos(planeController.copter))
        planeController.loose_jacohand()
        time.sleep(3)
        planeController.move_to([0,0,0.25])
        planeController.grap_jacohand()
        time.sleep(3)
        planeController.move_to([0,0.5,0.6])
        planeController.move_to([0,0.5,0.25])
        planeController.loose_jacohand()
        # planeController.down_gear()
        planeController.landing()
        time.sleep(30)



mainController = MainController()
mainController.startSimulation()