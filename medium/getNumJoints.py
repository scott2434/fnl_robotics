import pybullet as p
p.connect(p.DIRECT)
car = p.loadURDF('simplecar_completed.urdf')
number_of_joints = p.getNumJoints(car)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(car, joint_number)
    #print(info)#includes more data
    print(info[0], ": ", info[1])#for basic data
