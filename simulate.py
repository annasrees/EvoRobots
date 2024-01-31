import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random
from simulation import SIMULATION

# backAmplitude = (1)
# backFrequency = 10
# backPhaseOffset = np.pi / 4 #changed - step 48

# frontAmplitude = (1)
# frontFrequency = 10
# frontPhaseOffset = 0


# backLegSensorValues = np.zeros(1000)
# frontLegSensorValues = np.zeros(1000)

# sinVals = np.linspace(0, (2 * np.pi), 1000)
# backTargetAngles = np.zeros(len(sinVals))
# frontTargetAngles = np.zeros(len(sinVals))

# for i in range(len(sinVals)):
#     backTargetAngles[i] = backAmplitude * np.sin(backFrequency * sinVals[i] + backPhaseOffset)
#     frontTargetAngles[i] = frontAmplitude * np.sin(frontFrequency * sinVals[i] + frontPhaseOffset)

# #backleg
# backOutFile = open("../EvoRobots/data/backLegSensorData.npy", "wb") 
# np.save(backOutFile, backTargetAngles)
# #fronleg
# frontOutFile = open("../EvoRobots/data/frontLegSensorData.npy", "wb") 
# np.save(frontOutFile, frontTargetAngles)
# backOutFile.close()
# frontOutFile.close()
# # sinData = open("../EvoRobots/data/SinData.npy", "wb")
# # np.save(sinData, targetAngles)
# # sinData.close()

# for i in range(1000):
#     p.stepSimulation()
#     #adjusting targetAngles - assignment 5 step 40
    
#     backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
#     frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
#     pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
#                                 jointName = "Torso_BackLeg",
#                                 controlMode = p.POSITION_CONTROL,
#                                 targetPosition = backTargetAngles[i],
#                                 maxForce = 500)
#     pyrosim.Set_Motor_For_Joint(bodyIndex = robotId,
#                                 jointName = "Torso_FrontLeg",
#                                 controlMode = p.POSITION_CONTROL,
#                                 targetPosition = frontTargetAngles[i],
#                                 maxForce = 500)
#     time.sleep(1/60)
# p.disconnect()
simulation = SIMULATION()