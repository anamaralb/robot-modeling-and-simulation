import pybullet as p
import time
import pybullet_data
import math
import csv

robot_path = "/home/ana/Documents/Modelado/urdf/robot.urdf"
cube_path = "/home/ana/Documents/Modelado/urdf/cube.urdf"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,-1,2]

robot_startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF(robot_path,startPos, robot_startOrientation)

cube_startOrientation = p.getQuaternionFromEuler([0,0,0])
cubeId = p.loadURDF(cube_path,[0,4,0.5], cube_startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
     print("{} - {} - {}".format(p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8"), p.getJointInfo(robotId,j)[2]))

data = []

state = 0
return_flag = 0
CUBE = 1
GET_CUBE = 2
DROP_CUBE = 3
LATERAL = 4
BOX = 5
START = 6
FINISH = 7

start_target = [0,3.271,2.914]
cube_target = [0,4,0.7]
front_target = [0, 4.5, 1.5]  #2, 1, 2.75
lateral_target = [2, 1, 3.5]
box_target = [0,0.25,3.5] #0,0.25,2
drop_target = [0,0.15,2.485]

last_time = time.time()

try:
    p.addUserDebugText("X", cube_target, [20,0,0], 1)
    p.addUserDebugText("Y", box_target, [20,0,0], 1)
    p.addUserDebugText("Z", lateral_target, [20,0,0], 1)
    p.addUserDebugText("W", front_target, [20,0,0], 1)
    p.addUserDebugText("V", drop_target, [20,0,0], 1)
    p.addUserDebugText("U", start_target, [20,0,0], 1)
    
    wheels_ind = [29,32,37,41]
    arm_ind = [14,15,17,19,21]
    prismatic_joints = [23,24]

    last_position = p.getBasePositionAndOrientation(robotId)[0][1]
    p.changeDynamics(cubeId, -1, lateralFriction=5)
    init_time = time.time()

    for i in arm_ind:
        p.enableJointForceTorqueSensor(robotId, i, enableSensor=True)

    for i in prismatic_joints:
        p.enableJointForceTorqueSensor(robotId, i, enableSensor=True)

    def dataForces(time):
        totalForce = 0
        for i in arm_ind:
            jointsForce = p.getJointState(robotId, i)[2][0:3]
            jointForce = 0
            for j in range(3):
                jointForce += abs(jointsForce[j])

            totalForce += jointForce

        for i in prismatic_joints:
            jointsForce = p.getJointState(robotId, i)[2][0:3]
            jointForce = 0
            for j in range(3):
                jointForce += abs(jointsForce[j])
            
            totalForce += jointForce

        data.append([time, 7,totalForce])

    while True:
        # p.setRealTimeSimulation(1)

        if (time.time() - last_time > 0.01 and state != 0):
            dataForces(time.time() - init_time)
            last_time = time.time()
        
        if state == 0:
            p.setJointMotorControlArray(robotId,wheels_ind, p.VELOCITY_CONTROL, targetVelocities=[-2,-2,-2,-2], forces=[1,1,1,1])
            if (p.getLinkState(robotId,21)[0][1] > 3.25):
                p.setJointMotorControlArray(robotId,wheels_ind, p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0], forces=[60,60,60,60])
                state = CUBE

        if state == CUBE:
                cube_joint_positions = p.calculateInverseKinematics(robotId, 21, cube_target)
                positions = [cube_joint_positions[0], cube_joint_positions[1], cube_joint_positions[2], cube_joint_positions[3], cube_joint_positions[4]]
                p.setJointMotorControlArray(robotId,arm_ind, p.POSITION_CONTROL, targetPositions=positions, forces=[250,250,150,50,20], positionGains=[0.03,0.03,0.03,0.03,0.03], velocityGains=[2,2,2,2,2])

                if (cube_target[0] - 0.05 < p.getLinkState(robotId, 21)[0][0] <= cube_target[0] + 0.05 and cube_target[1] - 0.05 < p.getLinkState(robotId, 21)[0][1] <= cube_target[1] + 0.05 and cube_target[2] - 0.05 < p.getLinkState(robotId, 21)[0][2] <= cube_target[2] + 0.05):
                    p.setJointMotorControlArray(robotId,prismatic_joints, p.POSITION_CONTROL, targetPositions=[0.06,0.06], forces=[150,150], positionGains=[0.05,0.05], velocityGains=[2,2])
                    if (p.getJointState(robotId, 23)[0] > 0.045 and p.getJointState(robotId, 24)[0] > 0.045):
                        state = GET_CUBE
                     
        if state == GET_CUBE:
                front_joints_positions = p.calculateInverseKinematics(robotId, 21, front_target, targetOrientation=p.getQuaternionFromEuler([math.pi,0,math.pi/2]))
                front_positions = [front_joints_positions[0], front_joints_positions[1], front_joints_positions[2], front_joints_positions[3], front_joints_positions[4]]
                p.setJointMotorControlArray(robotId,arm_ind, p.POSITION_CONTROL, targetPositions=front_positions, forces=[250,250,250,50,50], positionGains=[0.05,0.05,0.05,0.05,0.05], velocityGains=[1.5,1.5,1.5,1.5,1.5])
                
                if (front_target[0] - 0.05 < p.getLinkState(robotId, 21)[0][0] <= front_target[0] + 0.05 and front_target[1] - 0.05 < p.getLinkState(robotId, 21)[0][1] <= front_target[1] + 0.05 and front_target[2] - 0.05 < p.getLinkState(robotId, 21)[0][2] <= front_target[2] + 0.05):
                    state = LATERAL

        if state == LATERAL:
                lateral_joints_positions = p.calculateInverseKinematics(robotId, 21, lateral_target, targetOrientation=p.getQuaternionFromEuler([math.pi,0,math.pi/2]))
                lateral_positions = [lateral_joints_positions[0], lateral_joints_positions[1], lateral_joints_positions[2], lateral_joints_positions[3], lateral_joints_positions[4]]
                p.setJointMotorControlArray(robotId,arm_ind, p.POSITION_CONTROL, targetPositions=lateral_positions, forces=[300,300,250,250,50], positionGains=[0.03,0.05,0.03,0.03,0.03], velocityGains=[2,2,2,2,2])

                if (lateral_target[0] - 0.2 < p.getLinkState(robotId, 21)[0][0] <= lateral_target[0] + 0.2 and lateral_target[1] - 0.2 < p.getLinkState(robotId, 21)[0][1] <= lateral_target[1] + 0.2 and lateral_target[2] - 0.2 < p.getLinkState(robotId, 21)[0][2] <= lateral_target[2] + 0.2):
                    if return_flag == 1:
                        state = START
                    else:
                        state = BOX

        if state == BOX:
                box_joints_positions = p.calculateInverseKinematics(robotId, 21, box_target, targetOrientation=p.getQuaternionFromEuler([math.pi,0,math.pi/2]))
                box_positions = [box_joints_positions[0], box_joints_positions[1], box_joints_positions[2], box_joints_positions[3], box_joints_positions[4]]
                p.setJointMotorControlArray(robotId,arm_ind, p.POSITION_CONTROL, targetPositions=box_positions, forces=[300,250,100,50,50], positionGains=[0.03,0.03,0.03,0.03,0.03], velocityGains=[2,2,2,2,2])

                if (box_target[0] - 0.3 < p.getLinkState(robotId, 21)[0][0] <= box_target[0] + 0.3 and box_target[1] - 0.3 < p.getLinkState(robotId, 21)[0][1] <= box_target[1] + 0.3 and box_target[2] - 0.3 < p.getLinkState(robotId, 21)[0][2] <= box_target[2] + 0.3):
                    if return_flag == 1:
                        state = LATERAL
                    else:
                        state = DROP_CUBE

        if state == DROP_CUBE:
                drop_joints_positions = p.calculateInverseKinematics(robotId, 21, drop_target, targetOrientation=p.getQuaternionFromEuler([math.pi,0,math.pi/2]))
                drop_positions = [drop_joints_positions[0], drop_joints_positions[1], drop_joints_positions[2], drop_joints_positions[3], drop_joints_positions[4]]
                p.setJointMotorControlArray(robotId,arm_ind, p.POSITION_CONTROL, targetPositions=drop_positions, forces=[300,300,200,50,50], positionGains=[0.03,0.03,0.03,0.03,0.03], velocityGains=[2,2,2,2,2])
                
                if (drop_target[0] - 0.05 < p.getLinkState(robotId, 21)[0][0] <= drop_target[0] + 0.05 and drop_target[1] - 0.05 < p.getLinkState(robotId, 21)[0][1] <= drop_target[1] + 0.05 and drop_target[2] - 0.05 < p.getLinkState(robotId, 21)[0][2] <= drop_target[2] + 0.05):
                    p.setJointMotorControlArray(robotId,prismatic_joints, p.POSITION_CONTROL, targetPositions=[0.0,0.0], forces=[30,30], positionGains=[0.03,0.03], velocityGains=[1.3,1.3])
                    if (p.getJointState(robotId, 23)[0] < 0.01 and p.getJointState(robotId, 24)[0] < 0.01):
                         state = BOX
                         return_flag = 1

        if state == START:
                start_joints_positions = p.calculateInverseKinematics(robotId, 21, start_target)
                start_positions = [start_joints_positions[0], start_joints_positions[1], start_joints_positions[2], start_joints_positions[3], start_joints_positions[4]]
                p.setJointMotorControlArray(robotId,arm_ind, p.POSITION_CONTROL, targetPositions=start_positions, forces=[200,200,100,100,20], positionGains=[0.03,0.03,0.03,0.03,0.03], velocityGains=[2,2,2,2,2])
                
                if (start_target[0] - 0.05 < p.getLinkState(robotId, 21)[0][0] <= start_target[0] + 0.05 and start_target[1] - 0.05 < p.getLinkState(robotId, 21)[0][1] <= start_target[1] + 0.05 and start_target[2] - 0.05 < p.getLinkState(robotId, 21)[0][2] <= start_target[2] + 0.05):
                    state = FINISH

        if state == FINISH:
                with open('dataP2.csv', 'w') as f:
                    writer = csv.writer(f)
                    writer.writerows(data)
                p.disconnect()
                exit(0)

        p.stepSimulation()
        time.sleep(1./240.)
        
except KeyboardInterrupt:
      pass
	
p.disconnect()    