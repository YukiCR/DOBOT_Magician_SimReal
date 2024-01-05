"""
    steps: \n
    0. init, connect and setHome \n
        assembleState:List[bool] = [False, False, False] \n
    1. if measure, measure some pose, \n
        else, load default pose \n
        pose: n row, 2 col
    2. while <there's obj>: \n
        wait for camera info, \n
            info:tuple[bool] = judgeInfo() \n

            for i in range(num): \n
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
        move to desination \n
        closeSuctionCup \n        
"""

# set USE_CVDECT to False if not using detect part
USE_CVDECT:bool = False
# set USE_SERIAL to False if not using serial
USE_SERIAL:bool = True

# import threading
import DobotDllType as dType # Dobot DLL for python
import json # import for saving and reading poses
from typing import List, Dict, Tuple # for type annotations and type hints
from enum import Enum, unique # for enum the `Speed` class
if USE_CVDECT:
    from CVDetect import CVDetector # CV API by Pi
if USE_SERIAL: # Pyserial for serial communication 
    import serial
    import serial.tools.list_ports


class manipulator:
    def __init__(self, autoAsync:bool = True, autoSetHome:bool=True) -> None:
        """
        do some inititlazation \n
        self.pose is a 3D list, where: \n
            dim 1: group number, typically 3 groups \n
            dim 2: point number, typically 2 points \n
            dim 3: a pose, which is p:List[float] = [X,Y,Z,R] in Cartesian space \n
        """
        self.api = dType.load()
        self.pose:List[List[ List[float] ]] = [] # 3D list
        self.assembleState:List[bool] = [False,False,False]
        self.path = 'poseLog.txt' # that's the file which will save measured poses
        self._label_dict = { # a dictionary mapping label numbers to corresponding names
            0: 'Ankle_Boot',
            1: 'Sandal',
            2: 'Sneaker',
        }
        # expected answer
        self.Ans:List[str] = [self._label_dict[0],
                              self._label_dict[1],
                              self._label_dict[2]]

        # try to connect dobot
        CON_STR = {
            dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
            dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
            dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
        state = dType.ConnectDobot(self.api, "", 115200)[0]
        print("Connect status:",CON_STR[state])
        # raise an error if fail to connect
        if (state != dType.DobotConnect.DobotConnect_NoError):
            raise ValueError("connection error")
        
        ### DO INIT ###
        #Clean Command Queued
        dType.SetQueuedCmdClear(self.api)
        
        #Async Motion Params Setting

        # set home, if not auto, home is set by hand-hold-teaching
        if autoSetHome:
            p:List[float] = [200,200,200,200] #default home
        else:
            dType.SetHHTTrigOutputEnabled(self.api, True) # enable trigger
            print("wait for set Home")
            while True: 
                if dType.GetHHTTrigOutput(self.api)[0] == True:# if set button on the arm
                    p = dType.GetPose(self.api) # get current pose
                    break # break loop
                pass
            dType.SetHHTTrigOutputEnabled(self.api, False) # disable trigger
        lastIndex:int = dType.SetHOMEParams(self.api, p[0], p[1], p[2], p[3], isQueued = 1)[0]
        print("home set successfully")
        print("home is:",p[0:3])

        # set PTP parameters    
        lastIndex = dType.SetPTPJointParams(self.api, 200, 200, 200, 200, 200, 200, 200, 200, isQueued = 1)[0]
        lastIndex = dType.SetPTPCommonParams(self.api, 30, 30, isQueued = 1)[0]
        lastIndex = dType.SetPTPJumpParams(self.api, 50, 230, isQueued = 1)[0] #jumpstep and max jump height

        # Start to Execute Command Queue
        dType.SetQueuedCmdStartExec(self.api)

        #Wait for Executing Last Command
        cnt:int = int(0) 
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(100)
            cnt += 1
            if cnt == 5:
                print("initializing ...")
                cnt = int(0)

        """
        CmdCurrentIndex would add 1 if the one command is done, yet \v
        dType.SetHOMECmd() don't, Somehow, it has to be done in this way \n
        to avoid another cmd to be execute before dType.SetHOMECmd() is finished \n 
        """
        if autoAsync:
            #Async Home
            lastIndex = dType.SetHOMECmd(self.api, temp = 0, isQueued = 1)[0]
        dType.SetQueuedCmdStopExec(self.api)

    def measure(self,num:int) -> List[List[float]]:
        """
        get some pose \n
        + buffer: 2D list, which is a list of poses,\
            it would be recorded \n
        + num: numbers of point to record \n
        by default, hand-hold-teaching would trigger if button is released. \n
        this method would automatically set manipulator to home after measureing
        """
        print("wait for hand-hold-teaching for",num,"poses")
        buffer:List[List[float]] = [] # 2D list
        dType.SetHHTTrigOutputEnabled(self.api, True) # enable trigger
        cnt:int = int(0)
        while cnt < num:
            if dType.GetHHTTrigOutput(self.api)[0] == True:
                buffer.append(dType.GetPose(self.api))
                print("pose", cnt+1 ,"measured: \n",buffer[-1])
                cnt += 1
        dType.SetHHTTrigOutputEnabled(self.api, False) # disable trigger
        # go back to home
        self.goHome()
        return buffer
    
    def buffer2pose(self,buffer:List[List[float]],autoSave=True) -> None:
        """
        reshape measured buffer to self.pose \n
        every col poses will be in the same group
        """
        self.pose = []
        col:int  = int(2)
        # n:int = int(0)
        # for i in range( round(len(buffer)/col + 0.5) ): #向上取整
        #     self.pose.append([]) # get pose[i] = []
        #     for j in range(col):
        #          self.pose[i].append(buffer[n])
        #          n += 1
        i:int = int(0)
        for p in buffer:
            if i == 0:
                self.pose.append([]) # new col
            self.pose[-1].append(p)
            i += 1
            if i == col:
                i = int(0)
        if autoSave:
            self.poseSave()
        pass

    def poseSave(self) -> None:
        """
        save pose to self.path, \n
        buffer2pose would automatically call this method \n
        """
        with open(self.path, 'w') as file:
            json.dump(self.pose, file)
        pass

    def poseLoad(self) -> None:
        """
        load pose from self.path, \n
        the way to get pose if not measure \n
        """
        with open(self.path, 'r') as file:
            self.pose = json.load(file)
        pass

    def judgeInfo(self,camInfo:List[str]) -> List[bool]:
        """
        judge of the camera info is corresponding to what we want
        """
        result:List[bool] = list(self.Ans[i]==camInfo[i] \
                    for i in range(min(len(self.Ans), len(camInfo))))
        return result
        pass

    def point2point(self,idx,sglAvoid=True) -> None:
        """
        read self.pose[idx] as poseCouple(idx start from 0)\n
        then do a set of movements: \n
        + wait \n
        + move to above of poseCouple[0] \n
        + open suction cup \n
        + catch poseCouple[0] \n
        + move to head of poseCouple[1] \n
        + go down to poseCouple[1] \n
        + close suction cup \n
        + go to head of the target \n
        **inputs**
        idx: self.pose[idx] will be use as a pose couple, 
            this method expect it to be 2 sets of poses. \n
        sglAvoid: if True, the trajactory is planed in joint space, \
            i.e., dType.PTPMode.PTPMOVJXYZMode is used for long traj; \n
            if Flase, plan in Cartesian space, the trajectory would 
            be a straight line in work space. Flase may lead to 
            singlularity problem.
        ```PYTHON
        # example
        r = manipulator(autoAsync=True, autoSetHome=True)
        r.buffer2pose(r.measure(2)) # release button to measure one pose
        r.point2point(0)
        ```
        """
        if len(self.pose) != 0:
            poseCouple:List[List[float]] = self.pose[idx]
            if len(poseCouple) != 2: # need to be pose couple
                pass
            else:
                # set diff as deltaY
                # set waittime as time to wait
                diff:float = 50
                waittime:float = 250
                # set mode
                if sglAvoid:
                    Mode = dType.PTPMode.PTPMOVJXYZMode
                else:
                    Mode = dType.PTPMode.PTPMOVLXYZMode
                """
                if the first cmd(i.e., wait) is deleted, goto head would be ignored
                which is a wierd problem
                -> this problem is not correlated to dobotExec function
                """
                # wait a little
                lastIndex = dType.SetWAITCmd(self.api, waittime, isQueued=1)[0] 
                #Set command queue
                p = poseCouple[0]
                # goto head of position
                lastIndex = dType.SetPTPCmd(self.api, Mode, \
                    p[0], p[1], p[2]+diff, p[3], isQueued = 1)[0] 
                # open suctioncup
                lastIndex = dType.SetEndEffectorSuctionCup(self.api, 1,  1, isQueued=1)[0] 
                # go down to position to catch, (this movement is samll, so PTPMOVLXYZMode is okay)
                lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, \
                    p[0], p[1], p[2], p[3], isQueued = 1)[0] 
                # wait for waittime, catch firm
                lastIndex = dType.SetWAITCmd(self.api, waittime, isQueued=1)[0]
                # go up to position's head again, this would avoid low obstaces
                lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, \
                    p[0], p[1], p[2]+diff, p[3], isQueued = 1)[0]
                """ deprecated temporarily
                # set p's X and Y same as target, so Z would not change
                # this may avoid collision
                p[0], p[1] = poseCouple[1][0], poseCouple[1][1]
                # goto [target[0], target[1], start[2], start[3]]
                lastIndex = dType.SetPTPCmd(self.api, Mode, \
                    p[0], p[1], p[2], p[3], isQueued = 1)[0] 
                """               
                # set p as the target position 
                p = poseCouple[1]
                """ 
                ! deprecated now
                # jump to position
                lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPJUMPXYZMode, \
                            p[0], p[1], p[2], p[3], isQueued = 1)[0]
                """
                # goto head of target from start, FAR
                lastIndex = dType.SetPTPCmd(self.api, Mode, \
                    p[0], p[1], p[2]+diff, p[3], isQueued = 1)[0]
                # goto target from target's head, (this movement is samll, so PTPMOVLXYZMode is okay)
                lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, \
                    p[0], p[1], p[2], p[3], isQueued = 1)[0]                
                # close suctioncup
                lastIndex = dType.SetEndEffectorSuctionCup(self.api, 0,  1, isQueued=1) 
                # wait a little, put down gentally
                lastIndex = dType.SetWAITCmd(self.api, waittime, isQueued=1)[0]
                # goto head of target from target, NEAR
                lastIndex = dType.SetPTPCmd(self.api, dType.PTPMode.PTPMOVLXYZMode, \
                    p[0], p[1], p[2]+diff, p[3], isQueued = 1)[0] 
                # wait a little
                lastIndex = dType.SetWAITCmd(self.api, waittime, isQueued=1)[0] 
                #Start to Execute Command Queue
                dType.SetQueuedCmdStartExec(self.api)
                #Wait for Executing Last Command
                cnt:int = int(0) 
                while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
                    dType.dSleep(100)
                    cnt += 1
                    if cnt == 10:
                        print("moving ...")
                        cnt = int(0)
                # Clear queue
                dType.SetQueuedCmdClear(self.api)
        else:
            pass
    
    def goHome(self,sglAvoid=True) -> None:
        """
        go to home form current pose. \n
        **input**
        sglAvoid: see point2point for this argument
        """
        # set mode
        if sglAvoid:
            Mode = dType.PTPMode.PTPMOVJXYZMode
        else:
            Mode = dType.PTPMode.PTPMOVLXYZMode        
        # wait
        lastIndex = dType.SetWAITCmd(self.api, 250, isQueued=1)[0]
        # get home position
        p = dType.GetHOMEParams(self.api)
        print("going back home: ",p)
        # go back to home
        lastIndex:int = dType.SetPTPCmd(self.api, Mode, \
            p[0], p[1], p[2], p[3], isQueued = 1)[0]
        # wait
        lastIndex = dType.SetWAITCmd(self.api, 500, isQueued=1)[0]
        #Start to Execute Command Queue
        dType.SetQueuedCmdStartExec(self.api)
        #Wait for Executing Last Command
        cnt:int = int(0) 
        while lastIndex > dType.GetQueuedCmdCurrentIndex(self.api)[0]:
            dType.dSleep(100)
            cnt += 1
            if cnt == 10:
                print("moving ...")
                cnt = int(0)
        #Clear queue
        dType.SetQueuedCmdClear(self.api)
        pass
 
    def manage(self,camInfo:List[str],judgeRes:List[bool]) -> None:
        result:Tuple[bool] = tuple(judgeRes)
        for idx,res in enumerate(result):
            if res == True and self.assembleState[idx] == False:
                # the right component and have not been assembled
                self.point2point(idx)
                self.assembleState[idx] = True
                print("component",idx,"assembled")
            else:
                pass
            if self.assembleState == [True,True,True]:
                print("one object assembled")
                self.assembleState = [False,False,False]
        dType.dSleep(1000) # sleep 1s
        pass



@unique
class Speed(Enum):
    """
    a enum of the speed of conveyor \n
    values should be unique by using @unique 
    """
    STOP = 0
    SLOW = 1
    MEDIUM = 2
    FAST = 3



class conveyor:
    """
    the conveyor class
    """
    def __init__(self) -> None:
        self.isOn:bool = False # whether a conveyor is on
        self.speed = Speed.STOP # current speed of the conveyor
        pass


######################################################################
# NOTE: hopes you check the states of CMC and send msg to CMC with this class 
#####################################################################
class CMC:
    """
    a class managing CMC, which will be slave of PC, \n
    ```
    """
    def __init__(self, cvyrNum:int = 3) -> None:
        """
        list all available serial ports, \n
        init serail connection with CMC, \n
        init a list of conveyors with <cvyrNum> elements \n
        ```PYTHON
        # example:
        c = CMC()
        # lookfor the first conveyor's state
        isFirstCvyrOn:bool = c.cvyr[0].isOn
        FirstCvyrSpeed:Speed = c.cvyr[0].speed
        FirstCvyrSpeedValue = c.cvyr[0].speed.value
        # check if Serial is open:
        isSerialOpen:bool = c.ser.isOpen()
        # use serial:
        c.ser.read(3) # read 3 bytes
        """
        # list avaliable serial devices
        ports_list = list(serial.tools.list_ports.comports())
        if len(ports_list) <= 0:
            print("no available serial device")
        else:
            print("available serial ports:")
            for comport in ports_list:
                print(list(comport)[0], list(comport)[1]) 
        # open serial port, open the port which is "CH340"
        # let the serial instance be self.ser
        self.ser = serial.Serial("COM11", 115200, timeout=None)
        if self.ser.isOpen():                       
            print("open",ser.name,"successfully")
        else:
            raise ValueError("open",ser.name,"failed")
        # set conveyors:
        # you can check all conveyors' states
        self.cvyr:List[conveyor] = [conveyor()] * cvyrNum
        pass
        
    def __del__(self) -> None:
        if self.ser.isOpen(): 
            # close the seial if it's open
            self.ser.close()
        if not self.ser.isOpen():
            print("Serial closed sucessfully")

    """TODO: a sets of methods, sending message to CMC with self.ser, changeing the states of self.cvyr""" 
    def TODO_send(self):
        # ........
        pass




if __name__ == "__main__":
    r = manipulator(autoAsync=True, autoSetHome=False)
    
    if USE_CVDECT:
        d = CVDetector()

    if USE_SERIAL:
        # list avaliable serial devices
        ports_list = list(serial.tools.list_ports.comports())
        if len(ports_list) <= 0:
            print("no available serial device")
        else:
            print("available serial ports:")
            for comport in ports_list:
                print(list(comport)[0], list(comport)[1]) 
        # open serial port, open the port which is "CH340"
        ser = serial.Serial("COM11", 115200, timeout=None)
        if ser.isOpen():                       
            print("open",ser.name,"successfully")
        else:
            raise ValueError("open",ser.name,"failed")        

    # r.buffer2pose(r.measure(6)) # measure pose, turn them to self.pose 
    r.poseLoad()
    print("Loaded")
    # print(len(r.pose), len(r.pose[1]), len(r.pose[0][0]))
    print("...")
    r.goHome()

    # for test
    # r.point2point(0)
    # r.point2point(1)
    # r.point2point(2)

    if USE_CVDECT and USE_SERIAL:
        while True:
            com_input = ser.read(3)
            hasObj:List[bool] = list(x == 1 for x in com_input)
            print("Serial receive:", hasObj)
            # if there's at least one obj accoarding to serial info
            if any(hasObj):
                # call cv
                CamInfo = d.classify()
                judgeRes:List[bool] = r.judgeInfo(CamInfo)
                print("CamInfo:",CamInfo)
                # if hasObj[i] == False, then judgeRes[i] would be False
                judgeRes =  [False if not a else b for a, b in zip(hasObj, judgeRes)]
                print("judgeRes:", judgeRes)
                r.manage(CamInfo,judgeRes)
                # would not go home after point2point, need call go home
                r.goHome()
                # clear all input buffer, only current msg will be received
                ser.reset_input_buffer()
            else:
                pass
            print("assemble state:", r.assembleState)
            print("---------------------------------------------")
        pass