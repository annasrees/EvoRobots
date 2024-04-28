import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
# NOTE TO SELF: original instrictions were to add "python3 generate.py". Changed to py instead
# for i in range(5):
#     os.system("py generate.py")
#     os.system("py simulate.py")
phcA = PARALLEL_HILL_CLIMBER("A")
print("phc A initialized")
phcA.Evolve()
print("\n phc A evolved. \n")
phcA.Show_best()
print("phc A has shown best. Complete")

phcB = PARALLEL_HILL_CLIMBER("B")
print("phc B initialized")
phcB.Evolve()
print("\n phc B evolved. \n")
phcB.Show_best()
print("phc B has shown best. Complete")


