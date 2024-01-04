from zmqRemoteApi import RemoteAPIClient
import math
import time

# a self-defined constant value
class Constant:
    def __setattr__(self, name, value):
        if name in self.__dict__:
            raise ValueError("Attempt to change a constant value")
        self.__dict__[name] = value


class IK:
    __sim = {'simTip':-1, 'simTarget':-1, 'simJoints':[]}
    __ik = {'ikEnv':-1, 'ikTarget':-1, 'ikBase':-1, 'ikJoints':[], 'ikGroup':-1}
    __scriptHandle = -1
    def __init__(self, tip, target, joints, scriptHandle, temp):
        self.__scriptHandle = scriptHandle
        self.__sim['simTip'],self.__sim['simTarget'],self.__sim['simJoints'] = tip,target,joints
        # self.__ik['ikEnv'], self.__ik['ikTarget'], self.__ik['ikBase'], self.__ik['ikJoints'], self.__ik['ikGroup']  \
        #       = sim.callScriptFunction('remoteApi_setIK',self.__scriptHandle,self.__sim)
        
        self.__ik['ikEnv'], self.__ik['ikTarget'], self.__ik['ikBase'], self.__ik['ikJoints'], self.__ik['ikGroup'] = temp
        print('seting... ik:', self.__ik['ikEnv'], self.__ik['ikTarget'], self.__ik['ikBase'], self.__ik['ikJoints'], self.__ik['ikGroup'])
        print(type(self.__ik['ikEnv']))
    def solve(self):
        print('solving... the ikEnv now is:', self.__ik['ikEnv'])
        jointPositions = sim.callScriptFunction('remoteApi_solveIK',self.__scriptHandle,self.__ik,self.__sim)
        return jointPositions
    def clean(self):
        print('cleaning... the ikEnv now is:', self.__ik['ikEnv'])
        sim.callScriptFunction('remoteApi_cleanIK',self.__scriptHandle,self.__ik)
        # del self
    def __del__(self):
        print('object deleted')
        

def norm_diff(a, b):
    # 计算列表长度
    n = len(a)
    
    # 计算欧几里得范数之差
    norm = 0
    for i in range(n):
        norm += (a[i] - b[i])**2
    norm = math.sqrt(norm)
    
    return norm


print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

# set the gravity, which would greatly influence the dynamic
sim.setArrayParameter(sim.arrayparam_gravity,[0,0,0])

executedMovId = 'notReady'
targetArm = '/Dobot'
objHandle = sim.getObject(targetArm)
print('handle: ', objHandle)
stringSignalName = targetArm + '_executedMovId'
scriptHandle = sim.getScript(sim.scripttype_childscript,objHandle)
print('script handle:', scriptHandle)


def waitForMovementExecuted(id_):
    global executedMovId, stringSignalName
    while executedMovId != id_:
        s = sim.getStringSignal(stringSignalName)
        executedMovId = s


# Set-up some movement variables:
maxVel = 0.1
maxAccel = 0.01
maxJerk = 80

# Start simulation:
sim.startSimulation()

# Wait until ready:
# loop until coppelisasim returns a 'ready'
waitForMovementExecuted('ready')

# get handles for motors and gripper
gripperHandle=sim.getObject('./suctionCup_link2')
motorHandles = []
for i in range(1,5):
    motorHandles.append( sim.getObject('./motor'+str(i)) )

simTip = sim.getObject('/Dobot/suctionCup/connection')
simTarget = sim.getObject('/ComparentCuboid')
DOF = 4
simJoints = []
for i in range(1,DOF+1):
    simJoints.append( sim.getObject('/Dobot/motor'+str(i)) )
motion = {
    'simTip': simTip,
    'simTarget': simTarget,
    'simJoints': simJoints
}
temp = sim.callScriptFunction('remoteApi_setIK',scriptHandle, motion)
IKup = IK(simTip, simTarget, simJoints, scriptHandle, temp)
# print('up', IKup.solve())

simTip = sim.getObject('/Dobot/suctionCup/connection')
simTarget = sim.getObject('/Cuboid[1]')
DOF = 4
simJoints = []
for i in range(1,DOF+1):
    simJoints.append( sim.getObject('/Dobot/motor'+str(i)) )
motion = {
    'simTip': simTip,
    'simTarget': simTarget,
    'simJoints': simJoints
}
temp = sim.callScriptFunction('remoteApi_setIK',scriptHandle, motion)
IKdown = IK(simTip, simTarget, simJoints, scriptHandle, temp)

#####
####
#### JUST SOLVE HERE
a = 1
print(type(a))
print('up', IKup.solve())
print('down', IKdown.solve())

IKup.clean()
IKdown.clean()


vel=22  
accel=40
jerk=80
movementData = {
    'motorHandles': motorHandles,
    'maxVel': [vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180],
    'maxAccel': [accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180],
    'maxJerk': [jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180],
    'targetConf':[],
    'enable': False
}


# conf  = [[50,0,0,45], [50,50,47,0], [50,0,0,45], [50,50,47,45], [50,0,0,45], [0,0,0,0]]
# trueOrFalse = [False, True, True, False, False, False]
# currentConf = [-1,-1,-1,-1]
# for i in range(len(conf)):
#     movementData['targetConf'],movementData['enable'] = conf[i],trueOrFalse[i]
#     # movementData['enable'] = trueOrFalse[i]
#     sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle,movementData)
#     # while norm_diff(currentConf,conf[i]) > 5:
#     #     for j in range(len(conf[i])):
#     #         currentConf[j]=sim.getJointPosition(motorHandles[j])*180/math.pi
#     time.sleep(1)


sim.stopSimulation()
print('Program ended')