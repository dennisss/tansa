#!/usr/bin/env python

# Used to nicely plot data from the `log` folder

import numpy as np
import matplotlib.pyplot as plt
import math

FILE = 'log/20170408-14_52_11.csv'

ROLE = 0

data = np.genfromtxt(FILE, delimiter=',', dtype=float)

#data = data[-10000:,:]

print(data.shape)


fig, axarr = plt.subplots(2, 1)

plot1 = axarr[0]
plot2 = axarr[1]

#plt.subplot(211)

off = 6*ROLE


# Target
plot1.plot(data[:, (off+1)], data[:, (off+2)], 'r--')

# Actual
plot1.plot(data[:, (off+4)], data[:, (off+5)], 'b')

#plt.plot(x, magX, 'b')
#plt.plot(x, p(current), 'r--')


plot1.set_ylim(-2, 2)
plot1.set_xlim(-3, 3)

plot1.legend(['Target Position', 'Actual Position'], prop={'size':9})
plot1.set_title('Position (meters)')

plot1.set_aspect('equal', adjustable='box')

#plt.subplot(212)

#plt.show()

e = data[:,(off+1):(off+4)] - data[:, (off+4):(off+7)]

plot2.plot(data[:, 0], e[:, 0], 'r')
plot2.plot(data[:, 0], e[:, 1], 'g')
plot2.plot(data[:, 0], e[:, 2], 'b')

plot2.set_ylim(-0.5, 0.5)
plot2.set_title('Axis Errors (meters)')
plot2.plot([0, data[-1, 0]], [0,0], '--', color='0.75')

plot2.legend(['X', 'Y', 'Z', 'Zero Line'], prop={'size':9})



fig.tight_layout()


plt.show()
#plt.plot(data[:,1:3],)
