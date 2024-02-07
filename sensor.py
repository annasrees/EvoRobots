import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np

class SENSOR:
    def __init__(self, linkName):
        self.linkName = linkName
        self.values = np.zeros(1000)

    def Get_Value(self, t):
        self.values[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        # if t == 1000:
        #     print(self.values)
        return self.values[t]

        
    def Save_Values(self):
        '''
        handles the saving of sensor values
        '''
        outFile = open("../EvoRobots/data/" + self.linkName + "Data.npy", "wb")
        np.save(outFile, self.values)
        outFile.close()

        
