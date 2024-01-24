import numpy as np
import matplotlib.pyplot as plt 

backLegSensorValues = np.load("../EvoRobots/data/backLegSensorData.npy")
print(backLegSensorValues)
plt.plot(backLegSensorValues)
plt.show()