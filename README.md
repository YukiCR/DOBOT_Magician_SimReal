# DOBOT_Magician_SimReal

a repsitory on basic simulation and control for 4 DOF manipulator DOBOT Magician with Python.
This repo has two parts, DobotSim and DobotReal, which is for simulation and control of DOBOT Magician.

- [DOBOT\_Magician\_SimReal](#dobot_magician_simreal)
  - [DobotSim](#dobotsim)
  - [DobotReal](#dobotreal)
  - [Download and Run](#download-and-run)
  - [Others](#others)


## DobotSim

![dobotsim](/Media/dobotsim.png)

We use Coppeliasim + Python for simulation of DOBOT Magician, these features are worth mentioned.

+ ZeroMQ API for python is adopted here, providing communication between coppeliasim and python
+ we are especially interested in the dynamics of robots in space, so the scene with gravity(DobotSim) and without gravity(DobotSimSpace) are both considered for comparision
+ Inverse kinamatics is solved in a numerical way using IK-plugin provided by coppeliasim
+ Simulation for non-gravity scene:
  The dynamics in space environment is very unstable, thus the error of of physics engine may lead to unexpected behavior(e.g., the robot in space scene may begin to move even no force is exert on it). Coppelisim allows user to select physics engine, we've tried bullet 2.78, bullet 2.83, ODE and Newton, and it seems that the physics engine provided by Newton Dynamics has the best stability.  
+ tons of challenging problems are still to be solved, including 
  + posture control of the base of the robot(i.e., a satellite probably) while the robot arm is working 
  + planning of the robot
  + a more complicated scene, we'll need a 6 DOF robot at least


## DobotReal

![RealPic](/Media/ManipulatorVideo.png)

DobotReal uses [Python API](https://www.dobot-robots.com/products/education/magician.html) provided by Yuejiang Robotics for secondary development.  
/DobotReal/robot.py provides a simple class `manipulator` for these functions:
+ measure poses with hand-hold-teaching
+ save and load poses got by hand-hold-teaching
+ moving a object from one point to another point, providing trajectory planned in *joint space* and *work space*.
+ communicating with [SUPCON](https://global.supcon.com/) CMC chip by serial and calling a CV classification method in class `CVDect`. [^1]
+ siiimple assembling management

[^1]: `CMC` part and `CVDetect` are accomplished by Su and Youzi correspondingly, so this part of code is not included in this repo. 

## Download and Run

+ if you are going to use serial, install [pySerial](https://pyserial.readthedocs.io/en/latest/pyserial.html) first.

+ clone this repo:
```
git clone https://github.com/YukiCR/DOBOT_Magician_SimReal.git
```
+ open the dir:
```
cd DOBOT_Magician_SimReal/
```
+ for DobotSim
    ```
    cd DobotSim/
    ```
  + open `/DobotSim/DobotSim.ttt` or `/DobotSim/DobotSimSpace.ttt` with Coppeliasim
  + run Python code for controlling manipulator in Coppeliasim:
    ```
    python basicControl.py
    ```
+ for DobotReal
    ```
    cd DobotSim/
    ```
  + add `/DobotReal` into your PATH
  + connect your PC with DOBOT Magician with USB and turn on the robot
  + run python code:
    ```
    python robot.py
    ```
  + this is an **example** of how do you use class `manipulator`:
    ```PYTHON
    import robot

    r = manipulator(autoAsync=False, autoSetHome=True) # init a instance, it would connect and init the manipulator
    buffer = r.measure(6) # measure 6 poses, release the button on arm to measure 1 pose
    r.buffer2pose(buffer) # give this buffer to class attribute
    # move from pose 1 to pose 2
    r.point2point(0)
    # move from pose 3 to 4
    r.point2point(1)
    # from 5 to 6
    r.point2point(2) 
    ```

## Others
+ Thanks to Automation Experiment Center,CSE,ZJU for providing instructions and equipments
+ Thanks to Su and Youzi for help
+ if you are interested or my code have bugs, you can contact me at <chengrui0103@foxmail.com>
+ if your INTERESTS is INFRIGEMENTED, DO CONTACT me at <chengrui0103@foxmail.com>
