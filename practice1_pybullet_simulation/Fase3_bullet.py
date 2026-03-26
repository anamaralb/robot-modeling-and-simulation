import pybullet as p
import time
import pybullet_data
import math
import csv

ramp_path = "ramp.urdf"
barrier_path = "barrier.urdf"
finish_path = "finish.urdf"
robot_path = "husky/husky.urdf"

physicsClient = p.connect(p.GUI) #or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

startPos = [0,0,0.5]
ramp_startOrientation = p.getQuaternionFromEuler([0,0,0])

planeId = p.loadURDF(ramp_path,startPos, ramp_startOrientation)

robot_startOrientation = p.getQuaternionFromEuler([0,0,math.pi/2])
robotId = p.loadURDF(robot_path,startPos, robot_startOrientation)

barrier_startOrientation = p.getQuaternionFromEuler([0,0,0])
barrierId = p.loadURDF(barrier_path,startPos, barrier_startOrientation)

finish_startOrientation = p.getQuaternionFromEuler([0,0,0])
finishId = p.loadURDF(finish_path,startPos, finish_startOrientation)

numJoints = p.getNumJoints(robotId)
print("NumJoints: {}".format(numJoints))
for j in range(numJoints):
     print("{} - {}".format(p.getJointInfo(robotId,j)[0], p.getJointInfo(robotId,j)[1].decode("utf-8")))

data = []
try:
    last_position = p.getBasePositionAndOrientation(robotId)[0][1]
    init_time = time.time()
    while True:
        p.setRealTimeSimulation(1)

        # Whit forces = [25],25,25,25] the robot can't go up the ramp
        p.setJointMotorControlArray(robotId,[2,3,4,5], p.VELOCITY_CONTROL, targetVelocities=[11,11,11,11], forces=[30,30,30,30])

        if (p.getBasePositionAndOrientation(robotId)[0][1] < 20.25):   
            if (p.getBasePositionAndOrientation(robotId)[0][1] - last_position > 0.01):
                data.append((time.time() - init_time, p.getBasePositionAndOrientation(robotId)[0][1], p.getBaseVelocity(robotId)[0][1] ,11,11,11,11, 30,30,30,30))
                last_position = p.getBasePositionAndOrientation(robotId)[0][1]

        # if (p.getBasePositionAndOrientation(robotId)[0][1] > 20.25):
        #     p.setJointMotorControlArray(robotId,[2,3,4,5], p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0], forces=[25,25,25,25])
        #     with open('data3_3.csv', 'w') as f:
        #         writer = csv.writer(f)
        #         writer.writerows(data)
        #     p.disconnect()
        #     exit(0)
        
except KeyboardInterrupt:
      pass
	
p.disconnect()    