import pyrosim.pyrosim as pyrosim

length = 1
width = 1
height = 1
x = 0
y = 0
z = .5

def Create_world():
    pyrosim.Start_SDF("world.sdf") 
    pyrosim.Send_Cube(name="Box", pos=[x + 4, y + 4, z] , size=[length, width, height]) #position changed so out of way; module E step 20   
    pyrosim.End()

def Create_Robot():
    pyrosim.Start_URDF("body.urdf")
    '''Storing a description of the robot's body here'''
    pyrosim.Send_Cube(name="Torso", pos=[0, 0, 0] , size=[length, width, height])
    pyrosim.Send_Joint( name = "Torso_Leg" , parent= "Torso" , child = "Leg" , type = "revolute", position = [0.5,0,0.75])
    pyrosim.Send_Cube(name="Leg", pos=[1.0,0,1.5] , size=[length, width, height])
    pyrosim.End()

Create_world()
Create_Robot()

