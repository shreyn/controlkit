import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

time_step = 0.1
cases = [
    ("push limit 0.5", "ball_mpc_push_only.csv"),
    ("push limit 0.5 and speed limit 1.0", "ball_mpc_speed.csv"),
]

# columns: tick, position, velocity, push
all_data = [np.loadtxt(csv_name, delimiter=",", skiprows=1) for _, csv_name in cases]
frame_ticks = list(range(0, len(all_data[0]), 2))  # draw every 2nd tick to keep the GIF small

figure, axes = plt.subplots(2, 1, figsize=(9, 5.5))
balls = []
push_arrows = []
info_texts = []

for axis, (title, _), data in zip(axes, cases, all_data):
    axis.set_xlim(-2.0, 10.6)
    axis.set_ylim(-1.0, 1.0)
    axis.set_yticks([])
    axis.set_title(title)
    axis.axhline(0.0, color="gray", linewidth=1)
    axis.plot([0.0], [0.0], marker="*", markersize=18, color="gold", markeredgecolor="black", linestyle="none")
    ball, = axis.plot([data[0, 1]], [0.0], marker="o", markersize=20, color="tab:blue")
    push_arrow = axis.annotate("", xy=(data[0, 1], -0.5), xytext=(data[0, 1], -0.5), arrowprops=dict(arrowstyle="-|>", linewidth=3, color="tab:red"))
    info_text = axis.text(0.02, 0.95, "", transform=axis.transAxes, va="top", family="monospace", fontsize=9)
    balls.append(ball)
    push_arrows.append(push_arrow)
    info_texts.append(info_text)

axes[1].set_xlabel("position (star = goal)")
figure.tight_layout()

def draw_frame(frame):
    tick = frame_ticks[frame]
    for i, data in enumerate(all_data):
        position = data[tick, 1]
        velocity = data[tick, 2]
        push = data[tick, 3]
        balls[i].set_data([position], [0.0])
        push_arrows[i].xy = (position + 2.0 * push, -0.5)
        push_arrows[i].set_position((position, -0.5))
        info_texts[i].set_text("tick %3d (t = %4.1f s)\nposition %7.3f\nvelocity %7.3f\npush     %7.3f" % (tick, tick * time_step, position, velocity, push))
    return balls + push_arrows + info_texts

animation = FuncAnimation(figure, draw_frame, frames=len(frame_ticks), interval=50, repeat=False)

animation.save("docs/ball_mpc.gif", writer="pillow", fps=20, dpi=100)

plt.show()