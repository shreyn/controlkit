import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# columns: tick, position, velocity, push
data = np.loadtxt("ball.csv", delimiter=",", skiprows=1)
ticks = data[:, 0]
positions = data[:, 1]
velocities = data[:, 2]
pushes = data[:, 3]
time_step = 0.1

figure, axis = plt.subplots(figsize=(9, 3.5))
axis.set_xlim(-1.0, 3.6)
axis.set_ylim(-1.0, 1.0)
axis.set_yticks([])
axis.set_xlabel("position")
axis.axhline(0.0, color="gray", linewidth=1)
axis.plot([0.0], [0.0], marker="*", markersize=18, color="gold", markeredgecolor="black", linestyle="none", label="goal (origin)")
axis.legend(loc="upper right")
figure.tight_layout()

ball, = axis.plot([positions[0]], [0.0], marker="o", markersize=22, color="tab:blue")
push_arrow = axis.annotate("", xy=(positions[0], 0.45), xytext=(positions[0], 0.45), arrowprops=dict(arrowstyle="-|>", linewidth=3, color="tab:red"))
info_text = axis.text(0.02, 0.95, "", transform=axis.transAxes, va="top", family="monospace")

def draw_frame(frame):
    position = positions[frame]
    velocity = velocities[frame]
    push = pushes[frame]
    ball.set_data([position], [0.0])
    push_arrow.xy = (position + 0.5 * push, 0.45)
    push_arrow.set_position((position, 0.45))
    info_text.set_text("tick     %3d  (t = %.1f s)\nposition %7.3f\nvelocity %7.3f\npush     %7.3f" % (ticks[frame], ticks[frame] * time_step, position, velocity, push))
    return ball, push_arrow, info_text

animation = FuncAnimation(figure, draw_frame, frames=len(ticks), interval=100, repeat=False)

animation.save("docs/ball.gif", writer="pillow", fps=10)

plt.show()