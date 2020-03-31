import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style

style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
ax2 = fig.add_subplot(1,2,1)
ax3 = fig.add_subplot(2,1,1)
ax4 = fig.add_subplot(2,2,1)
def animate(i):
    graph_data = open('log_data.csv','r').read()
    lines = graph_data.split('\n')
    xs = []
    q1s = []
    q2s = []
    q3s = []
    q4s = []
    for j, line in enumerate(lines):
        if len(line) > 1:
            q1, q2, q3, q4 = line.split(',')
            xs.append(j)
            q1s.append(float(q1))
            q2s.append(float(q2))
            q3s.append(float(q3))
            q4s.append(float(q4))
    ax1.clear()
    ax1.plot(xs, q1s)
    ax2.plot(xs, q2s)
    ax3.plot(xs, q3s)
    ax4.plot(xs, q4s)

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()
