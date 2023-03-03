import pybullet as p
from time import sleep
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
wheel_indices = [1, 3, 4, 5]#may need to go after sleep
hinge_indices = [0, 2]#may need to go after sleep
p.setGravity(0, 0, -10)
angle = p.addUserDebugParameter('Steering', -0.5, 0.5, 0)
#Sliders are added to our GUI view that allow us to dynamically input values to our program through the simulation
throttle = p.addUserDebugParameter('Throttle', -20, 20, 0)#low, high, starting speed
car = p.loadURDF('simplecar_completed.urdf', [0, 0, 0.1])
planeId = p.loadURDF("plane.urdf")
#plane = p.loadURDF('simpleplane.urdf')
block = p.loadURDF("simpleblock.5_5_.5.urdf", [1, 4, 0.1])
#cube = p.loadURDF('/~/medium/urdf_tutorial/robot.urdf')need to fin d syntax for impoting  urdf
#cube = p.loadURDF("/home/medium/onshape.urdf")
#goal = p.loadURDF('simplegoal.urdf',[0, 3, 0.5])
#shape = p.loadURDF('onshape2.urdf',[0, 1, 0.5])
sleep(3)

while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    for joint_index in wheel_indices:
        p.setJointMotorControl2(car, joint_index,
                                p.VELOCITY_CONTROL,
                                targetVelocity=user_throttle)
    for joint_index in hinge_indices:
        p.setJointMotorControl2(car, joint_index,
                                p.POSITION_CONTROL, 
                                targetPosition=user_angle)
    p.stepSimulation()
