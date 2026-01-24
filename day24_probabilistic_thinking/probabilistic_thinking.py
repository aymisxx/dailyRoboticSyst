import numpy as np
import matplotlib.pyplot as plt

mu = 5.0          # believed position
sigma = 0.7       # uncertainty

samples = np.random.normal(mu, sigma, 10000)

plt.hist(samples, bins=50, density=True)
plt.title("Belief over robot position")
plt.xlabel("Position")
plt.ylabel("Probability density")
plt.show()