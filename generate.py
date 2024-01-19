import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("world.sdf")

length = 1
width = 1
height = 1
x = 0
y = 0
z = .5
pyrosim.Send_Cube(name="Box", pos=[x, y, z] , size=[length, width, height])
# pyrosim.Send_Cube(name="Box2", pos=[x + 1, y, z + 1] , size=[length, width, height])
# for i in range(10):
#     length = length * 0.9
#     width = width * 0.9
#     height = height * 0.9
#     for j in range(5):
#         for k in range(5):
#             pyrosim.Send_Cube(name="Box" + str(i), pos=[x + j, y + k, z + i] , size=[length, width, height])
            

pyrosim.End()


