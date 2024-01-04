--[[
    The code for the dobot magician
    includes functions for api
    ik issues which uses coppeliasim ik-plugin
    and other nesessary issues
]]--


-- FUNCTIONS FOR API
function remoteApi_movementDataFunction(movData)
    local mh = movData.motorHandles
    local mv = movData.maxVel
    local ma = movData.maxAccel
    local mj = movData.maxJerk
    local target = movData.targetConf
    local en = movData.enable
    moveToConfig(mh,mv,mv,mj,target,en)
end

function remoteApi_setIK(simData)
    local tip = simData.simTip
    local target = simData.simTarget
    local joints = simData.simJoints
    local ikEnv,ikTarget,ikBase,ikJoints,ikGroup = setIK(tip,target,joints)
    
    print("API IK")
    print(ikEnv,ikTarget,ikBase,ikJoints)
    
    return ikEnv,ikTarget,ikBase,ikJoints,ikGroup
end

function remoteApi_solveIK(ikData,simData)
    local ikEnv = ikData.ikEnv
    local ikTarget = ikData.ikTarget
    local ikBase = ikData.ikBase
    local ikJoints = ikData.ikJoints
    local ikGroup = ikData.ikGroup
    local JointPositions = solveIK(ikEnv,ikTarget,ikBase,ikJoints,ikGroup,simData.simTarget)
    print("API ANS")
    print(JointPositions)
    return JointPositions
end

function remoteApi_cleanIK(ikData)
    print("DEL")
    print(ikData.ikEnv)
    simIK.eraseEnvironment(ikData.ikEnv)
end








-- IK ISSUES
function setIK(simTip, simTarget, simJoints)
-- INPUT: simTarget: the handle of the target
-- this would set the IK env if it goes well,
-- corresponding to the elaborate way of the tutorial
    simBase = sim.getObject('.')
    --local simTip = sim.getObject('/Dobot/suctionCup/connection')
    --local simTarget = sim.getObject('/Cuboid[0]')
    --local simJoints = {} -- joint in sim space
    local ikJoints = {} -- joint in ik space
    local ikEnv = simIK.createEnvironment()
    local ikBase = simIK.createDummy(ikEnv)
    -- set that dummy into the same pose as its CoppeliaSim counterpart: 
    simIK.setObjectMatrix(ikEnv,ikBase,-1,sim.getObjectMatrix(simBase,-1))
    local parent = ikBase
    for i = 1,#simJoints,1 do 
        -- create a joint in the IK environment:
        ikJoints[i]=simIK.createJoint(ikEnv,simIK.jointtype_revolute)
        -- set it into IK mode: 
        simIK.setJointMode(ikEnv,ikJoints[i],simIK.jointmode_ik)
        -- set the same joint limits as its CoppeliaSim counterpart joint: 
        local cyclic,interv=sim.getJointInterval(simJoints[i])
        simIK.setJointInterval(ikEnv,ikJoints[i],cyclic,interv)
        -- set the same joint lin./ang. position as its CoppeliaSim counterpart joint: 
        simIK.setJointPosition(ikEnv,ikJoints[i],sim.getJointPosition(simJoints[i]))
        -- set the same object pose as its CoppeliaSim counterpart joint: 
        simIK.setObjectMatrix(ikEnv,ikJoints[i],-1,sim.getObjectMatrix(simJoints[i],-1))
        -- set its corresponding parent: 
        simIK.setObjectParent(ikEnv,ikJoints[i],parent,true) 
        parent=ikJoints[i]
    end
    local ikTip = simIK.createDummy(ikEnv)
    -- set that dummy into the same pose as its CoppeliaSim counterpart: 
    simIK.setObjectMatrix(ikEnv,ikTip,-1,sim.getObjectMatrix(simTip,-1))
    -- attach it to the kinematic chain: 
    simIK.setObjectParent(ikEnv,ikTip,parent,true)
        -- create the target dummy in the IK environment: 
    local ikTarget=simIK.createDummy(ikEnv)
    -- set that dummy into the same pose as its CoppeliaSim counterpart: 
    simIK.setObjectMatrix(ikEnv,ikTarget,-1,sim.getObjectMatrix(simTarget,-1))
    -- link the two dummies: 
    simIK.setLinkedDummy(ikEnv,ikTip,ikTarget) -- use the deprecated setLinkedDummy here
    -- IK Group
    --[[
    ikGroup_undamped=simIK.createIkGroup(ikEnv) -- use the deprecated createIkGroup here
    -- set its resolution method to undamped: 
    simIK.setIkGroupCalculation(ikEnv,ikGroup_undamped,simIK.method_pseudo_inverse,0,6) -- the deprecated setIkGroupCalculation
    -- make sure the robot doesn't shake if the target position/orientation wasn't reached:  //always error, removed
    -- simIK.setGroupFlags(ikEnv,ikGroup_undamped,simIK.group_enabled|simIK.group_ignoremaxsteps|simIK.group_restoreonbadlintol|simIK.group_restoreonbadangtol)
    -- add an IK element to that IK group: 
    -- deprecated IkElements and IkElementBase here
    local ikElementHandle=simIK.addIkElement(ikEnv,ikGroup_undamped,ikTip)
    -- specify the base of that IK element: 
    simIK.setIkElementBase(ikEnv,ikGroup_undamped,ikElementHandle,ikBase)
    -- specify the constraints of that IK element: 
    simIK.setIkElementConstraints(ikEnv,ikGroup_undamped,ikElementHandle,simIK.constraint_pose)
    -- simIK.setIkElementConstraints(ikEnv,ikGroup_undamped,ikElementHandle,simIK.constraint_position)
    --]]
    -- create another IK group: 
    local ikGroup_damped=simIK.createIkGroup(ikEnv)
    -- set its resolution method to damped: 
    simIK.setIkGroupCalculation(ikEnv,ikGroup_damped,simIK.method_damped_least_squares,1,99)
    -- add an IK element to that IK group: 
    local ikElementHandle=simIK.addIkElement(ikEnv,ikGroup_damped,ikTip)
    -- specify the base of that IK element: 
    simIK.setIkElementBase(ikEnv,ikGroup_damped,ikElementHandle,ikBase)
    -- specify the constraints of that IK element: 
    simIK.setIkElementConstraints(ikEnv,ikGroup_damped,ikElementHandle,simIK.constraint_pose)
    -- simIK.setIkElementConstraints(ikEnv,ikGroup_damped,ikElementHandle,simIK.constraint_position)
    -- simIK.constraint_position
    return ikEnv,ikTarget,ikBase,ikJoints,ikGroup_damped
end

function solveIK(ikEnv,ikTarget,ikBase,ikJoints,ikGroup,simTarget)
    -- reflect the pose of the target dummy to its counterpart in the IK environment:
    simIK.setObjectMatrix(ikEnv,ikTarget,ikBase,sim.getObjectMatrix(simTarget,simBase)) 
    -- try to solve with the undamped method:
    -- use the deprecated handleIkGroup here
    --[[
    -- it would output IK solver failed here anyway, this is probably because simIK.result_success works for new versions
    if simIK.handleIkGroup(ikEnv,ikGroup_undamped)~=simIK.result_success then 
        -- the position/orientation could not be reached.
        -- try to solve with the damped method:
        simIK.handleIkGroup(ikEnv,ikGroup_damped)
        -- We display a IK failure report message: 
        -- sim.addLog(sim.verbosity_scriptwarnings,"IK undamped solver failed.") 
    end
    ]]--
    simIK.handleIkGroup(ikEnv,ikGroup)
    local JointPositions = {}
    for i = 1,#ikJoints,1 do
        JointPositions[i] = simIK.getJointPosition(ikEnv,ikJoints[i])
    end
    -- print(JointPositions)
    return JointPositions
end

function cleanIK(ikEnv)
    -- erase the IK obj: 
    simIK.eraseEnvironment(ikEnv) 
end











function sysCall_init()
    corout=coroutine.create(coroutineMain)
    -- send ready signal to python
    stringSignalName='/Dobot_executedMovId'
    sim.setStringSignal(stringSignalName,'ready')
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    else
        corout=coroutine.create(coroutineMain)
    end
end

function movCallback(config,vel,accel,handles)
    for i=1,#handles,1 do
        if sim.isDynamicallyEnabled(handles[i]) then
            sim.setJointTargetPosition(handles[i],config[i])
        else    
            sim.setJointPosition(handles[i],config[i])
        end
    end
end

function moveToConfig(handles,maxVel,maxAccel,maxJerk,targetConf,enable)
    stringSignalName='/Dobot_executedMovId'
    sim.setStringSignal(stringSignalName,'not done')
    
    local currentConf={}
    for i=1,#handles,1 do
        currentConf[i]=sim.getJointPosition(handles[i])
        targetConf[i]=targetConf[i]*math.pi/180
    end
    sim.moveToConfig(-1,currentConf,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,movCallback,handles)

    if enable then
        sim.writeCustomDataBlock(gripperHandle,'activity','on')
    else
        sim.writeCustomDataBlock(gripperHandle,'activity','off')
    end
    
    stringSignalName='/Dobot_executedMovId'
    sim.setStringSignal(stringSignalName,'done')
end


function coroutineMain()
    modelBase=sim.getObject('.')
    gripperHandle=sim.getObject('./suctionCup_link2')
    motorHandles = {}
    for i=1,4,1 do
        motorHandles[i]=sim.getObject('./motor'..i)
    end
    local vel=22
    vel = 50
    local accel=50
    local jerk=80
    local maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
    local maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
    local maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
    
    -- JUST FOR TEST
    --[[
    moveToConfig(motorHandles,maxVel,maxAccel,maxJerk,{50,0,0,45},false)
    moveToConfig(motorHandles,maxVel,maxAccel,maxJerk,{50,50,47,0},true)
    moveToConfig(motorHandles,maxVel,maxAccel,maxJerk,{50,0,0,45},true)
    moveToConfig(motorHandles,maxVel,maxAccel,maxJerk,{50,50,47,45},false)
    moveToConfig(motorHandles,maxVel,maxAccel,maxJerk,{50,0,0,45},false)
    moveToConfig(motorHandles,maxVel,maxAccel,maxJerk,{0,0,0,0},false)
    ]]--
end


