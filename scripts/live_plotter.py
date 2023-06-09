"""
author: Corrie Van Sice, 2023
"""

import time
import matplotlib.pyplot as _plt
import matplotlib.animation as _animation

class LivePlotter:
    
    xs = []
    ys = []

    _fig = _plt.figure()
    _ax = _fig.add_subplot(1, 1, 1)

    # def __init__(self):
    #     self.xs.append(0)
    #     self.ys.append(0)

    def update(self, x, y):

        self.xs.append(x)
        self.ys.append(y)

        # Limit x and y lists to 20 items
        self.xs = self.xs[-200:]
        self.ys = self.ys[-200:]
        print("update values")

        # Draw x and y lists
        self._ax.clear()
        self._ax.plot(xs, ys)

        # Format plot
        _plt.xticks(rotation=45, ha='right')
        _plt.subplots_adjust(bottom=0.30)
        _plt.title('Time')

    # This function is called periodically from FuncAnimation
    def animate(self, i, xs, ys):
        print("animate")
        # Draw x and y lists
        self._ax.clear()
        self._ax.plot(xs, ys)

        # Format plot
        _plt.xticks(rotation=45, ha='right')
        _plt.subplots_adjust(bottom=0.30)
        _plt.title('Time')
        _plt.ylabel('vel')

    def run(self, fn):
        print("run")
        anim = _animation.FuncAnimation(self._fig, fn, fargs=(self.xs, self.ys), interval=300)
        _plt.show()
        
    