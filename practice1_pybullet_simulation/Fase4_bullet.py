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


#vel_id = p.addUserDebugParameter("motor_vel", 0, 10, 0)
#left_id = p.addUserDebugParameter("leftMotor", -1, 1, 0)
data = []
try:
    last_position = p.getBasePositionAndOrientation(robotId)[0][1]
    init_time = time.time()
    velFront1 = 2.5
    velFront2 = 2.5
    velBack1 = 6.5
    velBack2 = 6.5

    forceFront1 = 10
    forceFront2 = 10
    forceBack1 = -17
    forceBack2 = -17

    while True:
        p.setRealTimeSimulation(1)
        
        orientation = p.getBasePositionAndOrientation(robotId)[1]
        angleY_rad = p.getEulerFromQuaternion(orientation)[1]
        angleY_deg = math.degrees(angleY_rad)

        velY = p.getBaseVelocity(robotId)[0][1]
        error = 2 - velY

        if (angleY_deg > 0 and angleY_deg < 1):
            targetVelocities = [10,10,11,11]
            forces = [20,20,30,30]

        elif (angleY_deg > 1 and angleY_deg < 15):
            targetVelocities = [10,10,6,6]
            forces = [20,20,30,30]

        elif (angleY_deg > 15 and angleY_deg < 20):
            targetVelocities = [1,1,0.75,0.75]
            forces = [0.5,0.5,5,5]

        elif (angleY_deg > 20 and angleY_deg < 26):
            targetVelocities = [-0.75,-0.75,1,1]
            forces = [10,10,10,10]

        elif (angleY_deg > 26 and angleY_deg < 32):
            targetVelocities = [-0.75,-0.75,-0.75,-0.75]
            forces = [10,10,10,10]

        elif (angleY_deg > 32 and angleY_deg < 33):
            targetVelocities = [-1,-1,-1,-1]
            forces = [20,20,20,20]

        elif (angleY_deg < -32 and angleY_deg > -18):
            targetVelocities = [4,4,10,10]
            forces = [15,15,30,30]

        elif (angleY_deg < -18 and angleY_deg > -6):
            targetVelocities = [4,4,10,10]
            forces = [20,20,40,40]

        elif (angleY_deg < -6 and angleY_deg > -2):
            targetVelocities = [4,4,6,6]
            forces = [20,20,30,30]

        elif (angleY_deg < -2 and angleY_deg > -1):
            targetVelocities = [6,6,8,8]
            forces = [25,25,15,15]

        elif (angleY_deg < -1 and angleY_deg > 0):
            targetVelocities = [4,4,8,8]
            forces = [25,25,15,15]
            
        else:
            targetVelocities = [11.5,11.5,11.5,11.5]
            forces = [30,30,30,30]

        

        p.setJointMotorControlArray(robotId,[2,3,4,5], p.VELOCITY_CONTROL, targetVelocities=targetVelocities, forces=forces)

        if (p.getBasePositionAndOrientation(robotId)[0][1] < 20.25):   
            if (p.getBasePositionAndOrientation(robotId)[0][1] - last_position > 0.01):
                data.append((time.time() - init_time, p.getBasePositionAndOrientation(robotId)[0][1], velY ,targetVelocities[0], targetVelocities[1], targetVelocities[2], targetVelocities[3], forces[0], forces[1], forces[2], forces[3], angleY_deg))
                last_position = p.getBasePositionAndOrientation(robotId)[0][1]

        if (p.getBasePositionAndOrientation(robotId)[0][1] > 20.25):
            p.setJointMotorControlArray(robotId,[2,3,4,5], p.VELOCITY_CONTROL, targetVelocities=[0,0,0,0], forces=[25,25,25,25])
            with open('data4.csv', 'w') as f:
                writer = csv.writer(f)
                writer.writerows(data)
            p.disconnect()
            exit(0)
        
except KeyboardInterrupt:
      pass
	
p.disconnect()    