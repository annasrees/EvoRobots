import numpy as np
import matplotlib.pyplot as plt 

backLegSensorValues = np.load("../EvoRobots/data/backLegSensorData.npy")
frontLegSensorValues = np.load("../EvoRobots/data/frontLegSensorData.npy")
# print(backLegSensorValues)

# sinValues = np.load("../EvoRobots/data/SinData.npy")
# plt.plot(sinValues)

plt.plot(backLegSensorValues, label = "back leg target angles", linewidth=4)
plt.plot(frontLegSensorValues, label = "front leg target angles")
plt.legend()
plt.show()