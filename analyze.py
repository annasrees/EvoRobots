import numpy as np
import matplotlib.pyplot as plt 

backLegSensorValues = np.load("../EvoRobots/data/backLegSensorData.npy")
frontLegSensorValues = np.load("../EvoRobots/data/frontLegSensorData.npy")
# print(backLegSensorValues)
plt.plot(backLegSensorValues, label = "back leg", linewidth=4)
plt.plot(frontLegSensorValues, label = "front leg")
plt.legend()
plt.show()