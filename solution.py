import numpy as np
import pyrosim.pyrosim as pyrosim
import os
class SOLUTION:
    def __init__(self):
        self.weights = np.random.rand(3,2)
        print(self.weights)
        self.weights = self.weights * 2 - 1
        print(self.weights)
        exit()


