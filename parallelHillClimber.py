from solution import SOLUTION
import constants as c
import copy
import os
import numpy as np
class PARALLEL_HILL_CLIMBER:
    def __init__(self, AB):
        os.system("del brain*.nndf")
        os.system("del Fitness*.nndf")
        os.system("del Fitness*.txt")
        populationSize = 10
        self.numberOfGenerations = 10 #changed from 10
        self.nextAvailableID = 0
        self.parents = {}
        self.AorB = AB
        self.ABMatrix = np.zeros((populationSize, self.numberOfGenerations))
        for i in range(populationSize):
            self.currentPop = i
            self.parents[i] = SOLUTION(self.nextAvailableID, self.AorB)
            self.nextAvailableID += 1
        # A/B Testing
        
        
    def Evolve(self):
        self.Evaluate(self.parents)     
        for currentGeneration in range(self.numberOfGenerations):
            self.Evolve_For_One_Generation(currentGeneration)
            currentGeneration += 1

            self.writeToABTest(self.ABMatrix)
        

    def Evolve_For_One_Generation(self, currentGeneration):
        self.Spawn()

        self.Mutate()

        self.Evaluate(self.children)

        self.Print()

        self.Select()

        for key in self.parents.keys():
            self.ABMatrix[key, currentGeneration] = self.parents[key].fitness

    def Spawn(self):
        self.children = {}
        for key in self.parents.keys():
            self.children[key] = copy.deepcopy(self.parents[key])
            self.children[key].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1
        # self.child = copy.deepcopy(self.parent)
        # self.child.Set_ID(self.nextAvailableID)
        # self.nextAvailableID += 1


    def Mutate(self):
        file = open("allRunABData.txt", "w")
        for key in self.parents.keys():
            self.children[key].Mutate()
            file.write(str(self.children[key].A_or_B) + "," + str(self.children[key].fitness) + "\n")
        # self.child.Mutate()
        file.close()

    def Select(self):
        file = open("bestParents.txt", "w")
        best_in_round = 0
        best_key = 0
        best_is_child = True
        for key in self.parents.keys():
            print("child: ", self.children[key].fitness)
            print("parent: ", self.parents[key].fitness)
            if(self.children[key].fitness > self.parents[key].fitness):
                self.parents[key] = self.children[key]
                print("parent: ", self.parents[key].fitness)
            if(self.children[key].fitness > best_in_round):
                best_in_round = self.children[key].fitness
                best_key = key
                best_is_child = True

            if(self.parents[key].fitness > best_in_round):
                best_in_round = self.parents[key].fitness
                best_key = key
                best_is_child = False
        if best_is_child == True:
            file.write(str(self.children[best_key].A_or_B) +  "," + str(self.children[best_key].fitness) + "\n")
        else:
            file.write(str(self.parents[best_key].A_or_B) +  "," + str(self.parents[best_key].fitness))

        file.close()



    def Print(self):
        print("\n")
        for key in self.parents.keys():
            print("Parent vs Child fitness: ", self.parents[key].fitness,  ", ",  self.children[key].fitness)
        print("\n")

    def Show_best(self):
        max_val = self.parents[0].fitness
        min_key = 0
        for key in self.parents.keys():
            fitness = self.parents[key].fitness
            # print(fitness)
            if fitness > max_val:
                max_val = key
                max_val = fitness
        self.parents[min_key].Start_Simulation("GUI")



    def Evaluate(self, solutions):
        for i in range(len(solutions)):
            solutions[i].Start_Simulation("DIRECT")

        for i in range(len(solutions)):
            solutions[i].Wait_For_Simulation_To_End()
            # print(solutions[i].fitness)

    def writeToABTest(self, ABMatrix):
        if self.AorB == "A":
            np.savetxt('AData.txt', ABMatrix)
            np.save('AData.npy', ABMatrix)
        else:
            np.savetxt('BData.txt', ABMatrix)
            np.save('BData.npy', ABMatrix)

        





        

