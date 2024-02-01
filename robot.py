from sensor import SENSOR
from motor import MOTOR
import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np

class ROBOT:
    def __init__(self, sensors, motors):
        # empty dictionaries
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)

        self.sensors = {}
        self.Prepare_To_Sense()

        self.motors = {}

    def Prepare_To_Sense(self):
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    '''
    t = current time step
    i = current sensor index
    '''
    def Sense(self, t):
        for i in self.sensors:
            self.sensors[i].Get_Value(t)
        # frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")

    def Prepare_To_Act(self):
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    def Act(self, t):
        for i in self.motors:
            self.motors[i].Set_Value(self.robot, t)