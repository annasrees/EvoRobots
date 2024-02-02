import pybullet as p
import pybullet_data
import time
import pyrosim.pyrosim as pyrosim
import numpy as np
import math
import random
from simulation import SIMULATION


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

# p.disconnect()
simulation = SIMULATION()
simulation.Run()
