import matplotlib.pyplot as plt
import numpy as np
import random

x = np.linspace(0, 100, 100)
y1 = np.linspace(0, 10, 100)
y2 = np.linspace(0, 10, 100)
y3 = np.linspace(0, 10000, 100)

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlabel('time (s)')
ax.set_ylabel('voltage', color='r')

ax2 = ax.twinx()
ax2.set_ylabel('current', color='g')

ax3 = ax.twinx()
ax3.set_ylabel('rpm', color='b')

line1, = ax.plot(x, y1, 'r-') # Returns a tuple of line objects, thus the comma
line2, = ax2.plot(x, y2, 'g-') # Returns a tuple of line objects, thus the comma
line3, = ax3.plot(x, y3, 'b-') # Returns a tuple of line objects, thus the comma
ax.grid()

for phase in np.linspace(0, 10*np.pi, 500):
    y1 = y1[1:]
    y1 = np.append(y1, random.randint(1,10))
    line1.set_ydata(y1)
    
    y2 = y2[1:]
    y2 = np.append(y2, random.randint(1,10))
    line2.set_ydata(y2)
    
    y3 = y3[1:]
    y3 = np.append(y3, random.randint(1,10000))
    line3.set_ydata(y3)
    
    #print(np.cos(x + phase))
    #line1.set_ydata(y1)
    #line1.set_ydata(np.cos(x + phase))
    fig.canvas.draw()
    fig.canvas.flush_events()
    