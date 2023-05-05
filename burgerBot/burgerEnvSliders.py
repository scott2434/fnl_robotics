import pybullet as p
import time
import pybullet_data
from time import sleep

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical >
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optiona>
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
wheel_indicesR = [0] #, 1]
wheel_indicesL = [1]
#boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
throttleR = p.addUserDebugParameter('ThrottleR', -50, 50, 0) #low,high,starting speed
throttleL = p.addUserDebugParameter('ThrottleL', -50, 50, 0)
carId = p.loadURDF("/home/reed/virEnv/med_Env3_15/bin/burgerBot/robot.urdf", basePosition=[0,0,0.2])
#set the center of mass frame (loadURDF sets base link frame) s>
while True:
   # user_angle = p.readUserDebugParameter(angle)
    user_throttleR = p.readUserDebugParameter(throttleR)
    for joint_index in wheel_indicesR:
        p.setJointMotorControl2(carId, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttleR)
    user_throttleL = p.readUserDebugParameter(throttleL)
    for joint_index in wheel_indicesL:
        p.setJointMotorControl2(carId, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttleL)

#       for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
