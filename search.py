import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
# NOTE TO SELF: original instrictions were to add "python3 generate.py". Changed to py instead
# for i in range(5):
#     os.system("py generate.py")
#     os.system("py simulate.py")
phc = PARALLEL_HILL_CLIMBER()
phc.Evolve()
phc.Show_best()
print("simulation complete!")


