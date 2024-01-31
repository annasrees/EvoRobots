from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim

class ROBOT:
    def __init__(self):
        # empty dictionaries
        '''
        31. cut the statements that load and prepare to simulate 
        body.urdf and paste them into ROBOT's constructor. 
        Remember to add the self. prefix where needed.
        '''
        self.robotId = p.loadURDF("body.urdf")
        self.sensors = {}
        self.motors = {}

