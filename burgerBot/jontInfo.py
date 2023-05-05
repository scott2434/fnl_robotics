import pybullet as p
p.connect(p.DIRECT)

#car = p.loadURDF('burgerBot.urdf')
car = p.loadURDF("/home/reed/virEnv/med_Env3_15/bin/burgerBot/robot.urdf")
number_of_joints = p.getNumJoints(car)
for joint_number in range(number_of_joints):
    info = p.getJointInfo(car, joint_number)
    print(info[0], ": ", info[1])

#p.setAdditionalSearchPath(pybullet_data.getDataPath())
#p.connect(p.GUI) #(p.DIRECT)
#car = p.loadURDF("/home/reed/virEnv/med_Env3_15/bin/burgerBot/robot.urdf")
#number_of_joints = p.getNumJoints(car)
#for joint_number in range(number_of_joints):
 #   info = p.getJointInfo(car, joint_number)
 #   print(info)
 #   print(info[0], ": ", info[1])
