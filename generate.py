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
    #come back to Module 3, instruction 31
    pyrosim.Send_Cube(name="Link0", pos=[x, y, z] , size=[length, width, height]) #torso L0
    pyrosim.Send_Joint( name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [0, 0, 1.0]) #torso_leg joint L0_L1
    pyrosim.Send_Cube(name="Link1", pos=[x, y, z] , size=[length, width, height]) #leg L1
    pyrosim.Send_Joint( name = "Link1_Link2" , parent= "Link1" , child = "Link2" , type = "revolute", position = [0, 0, 1.0]) #joint L1_l2
    pyrosim.Send_Cube(name="Link2", pos=[x, y, z] , size=[length, width, height]) #L2
    pyrosim.Send_Joint( name = "Link2_Link3" , parent= "Link2" , child = "Link3" , type = "revolute", position = [0, 0.5, 0.5]) #joint L2_l3
    pyrosim.Send_Cube(name="Link3", pos=[0, 0.5, 0] , size=[length, width, height]) #L3
    pyrosim.End()

Create_world()
Create_Robot()

