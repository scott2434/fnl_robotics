import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical >
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optiona>
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
mazeId = p.loadURDF("PUT YOUR FILE LOCATION HERE.urdf", basePosition=[0,0,0])
#boxId = p.loadURDF("r2d2.urdf",startPos, startOrientation)
#carId = p.loadURDF("/home/reed/ros2_ws_1.1/src/urdf_car_body/robot.urdf", basePosition=[0,0,0.2])
#set the center of mass frame (loadURDF sets base link frame) s>
while True:
#       for i in range (10000):
    p.stepSimulation()
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()

