import matplotlib.pyplot as plt

fig = plt.figure()

ax1 = fig.add_subplot(1, 1, 1)

X = [0, 1, 2]
Y = [0, 1, 2]
ax1.plot(X, Y)

plt.show()