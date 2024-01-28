import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)
 
for i in range(1000):
    p.stepSimulation()
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
                                jointName = "Torso_BackLeg",
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = np.radians(random.uniform((-2 * (math.pi)/1.0),2 * (math.pi)/1.0)),
                                maxForce = 500)
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
                                jointName = "Torso_FrontLeg",
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = np.radians(random.uniform((-(math.pi)/2),(math.pi)/2)),
                                maxForce = 500)
    time.sleep(1/60)
p.disconnect()
#backleg
outFile = open("../EvoRobots/data/backLegSensorData.npy", "wb") 
np.save(outFile, backLegSensorValues)
#fronleg
outFile = open("../EvoRobots/data/frontLegSensorData.npy", "wb") 
np.save(outFile, frontLegSensorValues)
outFile.close()

print(backLegSensorValues)
