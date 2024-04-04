from solution import SOLUTION
import constants as c
import copy
import os
class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del Fitness*.nndf")
        populationSize = 10
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1
        # self.parent = SOLUTION()
        
    def Evolve(self):
        self.Evaluate(self.parents)

        # for i in range(len(self.parents)):
        #     self.parents[i].Start_Simulation('DIRECT')
        # self.parent.Evaluate('GUI')
        numberOfGenerations = 10
        for currentGeneration in range(numberOfGenerations):
            self.Evolve_For_One_Generation()
            currentGeneration += 1
        # for i in range(len(self.parents)):
        #     self.parents[i].Wait_For_Simulation_To_End()
        #     print(self.parents[i].fitness)

    def Evolve_For_One_Generation(self):
        self.Spawn()

        self.Mutate()

        self.Evaluate(self.children)

        self.Print()

        self.Select()

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
        for key in self.parents.keys():
            self.children[key].Mutate()
        # self.child.Mutate()

    def Select(self):
        for key in self.parents.keys():
            if(self.children[key].fitness > self.parents[key].fitness):
                self.parents[key] = self.children[key]

    def Print(self):
        print("\n")
        for key in self.parents.keys():
            print("Parent vs Child fitness: ", self.parents[key].fitness,  ", ",  self.children[key].fitness)
        print("\n")

    def Show_best(self):

        min_val = self.parents[0].fitness
        min_key = 0
        for key in self.parents.keys():
            fitness = self.parents[key].fitness
            # print(fitness)
            if fitness < min_val:
                min_key = key
                min_val = fitness
        self.parents[min_key].Start_Simulation("GUI")


    def Evaluate(self, solutions):
        for i in range(len(solutions)):
            solutions[i].Start_Simulation("DIRECT")

        for i in range(len(solutions)):
            solutions[i].Wait_For_Simulation_To_End()
            # print(solutions[i].fitness)




        

