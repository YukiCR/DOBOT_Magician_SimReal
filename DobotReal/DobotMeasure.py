"""
    steps: \n
    0. init, connect and setHome \n
        assembleState:List[bool] = [False, False, False] \n
    1. if measure, measure some pose, \n
        else, load default pose \n
        pose: n row, 2 col
    2. while loop: \n
        wait for camera info, \n
            info:tuple[bool] = judgeInfo() \n

            for i in range(num):
                if info[i] == True and assembleState[i] == False: \n
                    move(i) \n
                    updateState(assembleState) \n
                else: \n
                    pass \n

            if assembleState == [True, True, True]: \n
                sleep ... \n
                assembleState = [False, False, False] \n
    in move: \n
        go to above \n
        openSuctionCup \n
        go down to catch \n
        JUMP to desination \n
        closeSuctionCup \n        
"""

import threading
import DobotDllType as dType
from typing import List, Dict, Tuple


class manipulator:
    def __init__(self) -> None:
        self.api = dType.load()
        self.pose:List[List[float]] = [] # 2D list

        # try to connect dobot
        CON_STR = {
            dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
            dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
            dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
        state = dType.ConnectDobot(api, "", 115200)[0]
        print("Connect status:",CON_STR[state])
        # rais an error if fail to connect
        if (state != dType.DobotConnect.DobotConnect_NoError):
            raise ValueError("connection error")
        
        # do init
        #Clean Command Queued
        dType.SetQueuedCmdClear(api)
        
        #Async Motion Params Setting
        lastIndex:int = dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued = 1)[0]
        lastIndex = dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)[0]
        lastIndex = dType.SetPTPCommonParams(api, 50, 50, isQueued = 1)[0]
        lastIndex = dType.SetPTPJumpParams(api, 50, 230, isQueued = 1)[0] #jumpstep and max jump height

        #Async Home
        lastIndex = dType.SetHOMECmd(api, temp = 0, isQueued = 1)[0]

        #Start to Execute Command Queue
        dType.SetQueuedCmdStartExec(api)

        #Wait for Executing Last Command
        cnt:int = 0 
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)
            cnt += 1
            if cnt == 10:
                print("initializing ...")
                cnt = 0



pose = [[245.1611328125, 96.15018463134766, 61.58674621582031, 21.414705276489258], 
        [243.94578552246094, -98.28384399414062, 62.75279235839844, -21.94411849975586],
        [245.1611328125, 96.15018463134766, 61.58674621582031, 21.414705276489258],
        [243.94578552246094, -98.28384399414062, 62.75279235839844, -21.94411849975586]]


#将dll读取到内存中并获取对应的CDLL实例
#Load Dll and get the CDLL object
api = dType.load()
#建立与dobot的连接
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
#Connect Dobot
state = dType.ConnectDobot(api, "", 115200)[0]
print("Connect status:",CON_STR[state])

if (state == dType.DobotConnect.DobotConnect_NoError):

    # For Measure    
    # while True:
    #     pose = dType.GetPose(api)
    #     print(pose)
    #     dType.dSleep(1000)
    
    #清空队列
    #Clean Command Queued
    dType.SetQueuedCmdClear(api)
    
    #设置运动参数
    #Async Motion Params Setting
    dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPJointParams(api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)
    dType.SetPTPCommonParams(api, 50, 50, isQueued = 1)
    dType.SetPTPJumpParams(api, 50, 230, isQueued = 1) #jumpstep and max jump height

    #回零
    #Async Home
    dType.SetHOMECmd(api, temp = 0, isQueued = 1)

    
    lastIndex = dType.SetEndEffectorSuctionCup(api, 1,  1, isQueued=1)
    p = pose[0]
    lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, p[0], p[1], p[2], p[3], isQueued = 1)[0]
    for i in range(2):
        p = pose[1]
        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, p[0], p[1], p[2], p[3], isQueued = 1)[0]
        lastIndex = dType.SetEndEffectorSuctionCup(api, 0,  1, isQueued=1)
        lastIndex = dType.SetWAITCmd(api, 1000, isQueued=1)
        lastIndex = dType.SetEndEffectorSuctionCup(api, 1,  1, isQueued=1)
        p = pose[0]
        lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPJUMPXYZMode, p[0], p[1], p[2], p[3], isQueued = 1)[0]


        
        

    #开始执行指令队列
    #Start to Execute Command Queue
    dType.SetQueuedCmdStartExec(api)

    #如果还未完成指令队列则等待
    #Wait for Executing Last Command 
    while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

    # close suctioncup
    dType.SetEndEffectorSuctionCup(api, 0,  1, isQueued=0)

    #停止执行指令
    #Stop to Execute Command Queued
    dType.SetQueuedCmdStopExec(api)


#断开连接
#Disconnect Dobot
dType.DisconnectDobot(api)
