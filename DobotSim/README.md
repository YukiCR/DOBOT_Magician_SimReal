# DobotSim
a simple project on CoppeliaSim + Python

+ ZeroMQ API for python is adopted here, providing communication between coppeliasim and python
+ some simple dynamics of Dobot Magician, a simple 4 DOF robot is simulated here
+ we are especially interested in the dynamics of robots in space, so the scene with gravity(DobotSim) and without gravity(DobotSimSpace) are both considered for comparision
+ Inverse kinamatics is solved in a numerical way using IK-plugin provided by coppeliasim
+ Simulation for non-gravity scene:
  The dynamics in space environment is very unstable, thus the error of of physics engine may lead to unexpected behavior(e.g., the robot in space scene may begin to move even no force is exert on it). Coppelisim allows user to select physics engine, we've tried bullet 2.78, bullet 2.83, ODE and Newton, and it seems that the physics engine provided by Newton Dynamics has the best stability.  
+ tons of challenging problems are still to be solved, including 
  + posture control of the base of the robot(i.e., a satellite probably) while the robot arm is working 
  + planning of the robot
  + a more complicated scene, we'll need a 6 DOF robot at least

feel free to contact us if you are interested