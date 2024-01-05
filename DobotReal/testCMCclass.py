import robot

c = robot.CMC()
# lookfor the first conveyor's state
isFirstCvyrOn:bool = c.cvyr[0].isOn
print("isOn:",isFirstCvyrOn)
FirstCvyrSpeed:robot.Speed = c.cvyr[0].speed
print("speed:",FirstCvyrSpeed)

# change speed
print("accelerate")
c.cvyr[0].speed = robot.Speed.FAST
print("speed now",c.cvyr[0].speed)
isFirstCvyrOn:bool = c.cvyr[0].isOn
print("isOn now:",isFirstCvyrOn)

# get speed value
FirstCvyrSpeedValue = c.cvyr[0].speed.value
print(FirstCvyrSpeedValue)
# check if Serial is open:
isSerialOpen:bool = c.ser.isOpen()
print(isSerialOpen)


c.readMsg()
c.changeSpeed(1, True)
