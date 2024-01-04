from zmqRemoteApi import RemoteAPIClient
import math
import time

""""""""""
Create more than 1 of this IK Class would lead to unexpected error, this is might because sim and function handles are used in the class
"""""""""
class IK:
    __sim = {'simTip':-1, 'simTarget':-1, 'simJoints':[]}
    __ik = {'ikEnv':-1, 'ikTarget':-1, 'ikBase':-1, 'ikJoints':[], 'ikGroup':-1}
    __scriptHandle = -1
    def __init__(self,simdata,ikdata,scriptHandle):
        self.__scriptHandle = scriptHandle
        self.__sim['simTip'],self.__sim['simTarget'],self.__sim['simJoints'] = simdata
        self.__ik['ikEnv'], self.__ik['ikTarget'], self.__ik['ikBase'], self.__ik['ikJoints'], self.__ik['ikGroup'] = ikdata
        print('seting... ik:', self.__ik['ikEnv'], self.__ik['ikTarget'], self.__ik['ikBase'], self.__ik['ikJoints'], self.__ik['ikGroup'])
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
    """
    a simple function calculating the difference between two vectors 
    """
    n = len(a)
    norm = 0
    for i in range(n):
        norm += (a[i] - b[i])**2
    norm = math.sqrt(norm)
    return norm



print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

# set the gravity, which would greatly influence the dynamic
sim.setArrayParameter(sim.arrayparam_gravity,[0,0,-9.81])

# set degree of freedom
DOF = 4

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


def moveToSet(motorHandles, target, enable):
    """
    move motor to set target \n
    loop until arrive at the setpoint \n
    values scriptHandleshould be global
    target should be in radius
    """
    # init movementData
    vel=22  
    accel=40
    jerk=80
    movementData = {
        'motorHandles': motorHandles,
        'maxVel': [vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180],
        'maxAccel': [accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180],
        'maxJerk': [jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180],
        'targetConf': [x*180/math.pi for x in target],  # trans from radius to degree here
        'enable': enable
    }
    sim.callScriptFunction('remoteApi_movementDataFunction',scriptHandle,movementData)
    # for i in range(len(motorHandles)):
    #     sim.setJointPosition(motorHandles[i],target[i])
    # map(lambda h,sp: sim.setJointPosition(h,sp), motorHandles, target)
    # currentConf = [None]*len(motorHandles)
    while True:
        currentConf = list( map(lambda h: sim.getJointPosition(h), motorHandles) ) # a very concise expression
        diff = norm_diff(currentConf, target)
        if diff > 0.01:
            print('diff:', norm_diff(currentConf, target))
        else:
            break


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
motorHandles = []
for i in range(1,DOF+1):
    motorHandles.append( sim.getObject('./motor'+str(i)) )

# create IK obj
class IK:
    sim = None
    ik = None
    pass

def setIK(tip, target):
    simTip = sim.getObject(tip)
    simTarget = sim.getObject(target)
    simJoints = []
    for i in range(1,DOF+1):
        simJoints.append( sim.getObject('/Dobot/motor'+str(i)) )
    simdata = {'simTip':simTip, 'simTarget':simTarget, 'simJoints':simJoints}
    ikdata = {'ikEnv':-1, 'ikTarget':-1, 'ikBase':-1, 'ikJoints':[], 'ikGroup':-1}
    ikdata['ikEnv'], ikdata['ikTarget'], ikdata['ikBase'], ikdata['ikJoints'], ikdata['ikGroup'] \
        = sim.callScriptFunction('remoteApi_setIK',scriptHandle,simdata)
    ik = IK()
    ik.sim = simdata
    ik.ik = ikdata
    return ik
    pass

time.sleep(0.5)

ikup = setIK(tip='/Dobot/suctionCup/connection', target='/ComparentCuboidUp')
ikobj = setIK(tip='/Dobot/suctionCup/connection', target='/ComparentCuboid')
iktarget = setIK(tip='/Dobot/suctionCup/connection', target='/ComparentTarget')


ikans = sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikup.ik, ikup.sim)
print('jp1', ikans)
moveToSet(motorHandles, ikans, False)
# time.sleep(1)

ikans = sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikobj.ik, ikobj.sim)
print('jp2:', ikans)
moveToSet(motorHandles, ikans, True)
time.sleep(1)

ikans = sim.callScriptFunction('remoteApi_solveIK',scriptHandle,ikup.ik, ikup.sim)
print('jp3:', ikans)
moveToSet(motorHandles, ikans, True)
time.sleep(0.5)

ikans = sim.callScriptFunction('remoteApi_solveIK',scriptHandle,iktarget.ik, iktarget.sim)
print('jp3:', ikans)
moveToSet(motorHandles, ikans, True)
time.sleep(0.5)

moveToSet(motorHandles, ikans, False)
time.sleep(0.5)


# for test
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


sim.callScriptFunction('remoteApi_cleanIK',scriptHandle,ikup.ik)
sim.callScriptFunction('remoteApi_cleanIK',scriptHandle,ikobj.ik)
sim.callScriptFunction('remoteApi_cleanIK',scriptHandle,iktarget.ik)
sim.stopSimulation()
print('Program ended')