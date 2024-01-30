import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random

amplitude = (np.pi / 4)
frequency = 1
phaseOffset = 0

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)
backLegSensorValues = np.zeros(1000)
frontLegSensorValues = np.zeros(1000)

sinVals = np.linspace(0, (2 * np.pi), 1000)
targetAngles = np.zeros(1000)
for i in range(len(sinVals)):
    targetAngles[i] = amplitude * np.sin(frequency * sinVals[i] + phaseOffset)
    print(targetAngles[i])


sinData = open("../EvoRobots/data/SinData.npy", "wb")
np.save(sinData, targetAngles)
sinData.close()

 
for i in range(1000):
    p.stepSimulation()
    #adjusting targetAngles - assignment 5 step 40
    
    backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
                                jointName = "Torso_BackLeg",
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = targetAngles[i],
                                maxForce = 500)
    pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
                                jointName = "Torso_FrontLeg",
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = targetAngles[i],
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
