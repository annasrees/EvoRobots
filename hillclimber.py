from solution import SOLUTION
import constants as c
import copy
class HILL_CLIMBER:
    def __init__(self):
        self.parent = SOLUTION()
        
    def Evolve(self):
        self.parent.Evaluate('GUI')
        numberOfGenerations = 10
        for currentGeneration in range(numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()

        self.Mutate()

        self.child.Evaluate('DIRECT')

        self.Print()

        self.Select()

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if(self.child.fitness > self.parent.fitness):
            self.parent = self.child

    def Print(self):
        print("Parent vs Child fitness: ", self.parent.fitness,  ", ",  self.child.fitness)

    def Show_best(self):
        self.parent.Evaluate('GUI')


        

