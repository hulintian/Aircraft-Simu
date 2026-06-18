import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

src = input("input")
# ===== 1. load data =====
df = pd.read_csv(src)

mx = df["missile_x_ecef_m"].values
my = df["missile_y_ecef_m"].values
mz = df["missile_z_ecef_m"].values

tx = df["target_x_ecef_m"].values
ty = df["target_y_ecef_m"].values
tz = df["target_z_ecef_m"].values

# ===== 2. figure =====
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_title("Missile Guidance Simulation (ECEF)")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# 轨迹线
missile_line, = ax.plot([], [], [], lw=2, label="Missile")
target_line, = ax.plot([], [], [], lw=2, label="Target")

# 当前点
missile_point, = ax.plot([], [], [], 'ro')
target_point, = ax.plot([], [], [], 'bo')

ax.legend()

# ===== 3. init =====
def init():
    ax.set_xlim(np.min(mx), np.max(mx))
    ax.set_ylim(np.min(my), np.max(my))
    ax.set_zlim(np.min(mz), np.max(mz))
    return missile_line, target_line

# ===== 4. update =====
def update(i):
    missile_line.set_data(mx[:i], my[:i])
    missile_line.set_3d_properties(mz[:i])

    target_line.set_data(tx[:i], ty[:i])
    target_line.set_3d_properties(tz[:i])

    missile_point.set_data([mx[i]], [my[i]])
    missile_point.set_3d_properties([mz[i]])

    target_point.set_data([tx[i]], [ty[i]])
    target_point.set_3d_properties([tz[i]])

    return missile_line, target_line, missile_point, target_point

# ===== 5. animation =====
ani = FuncAnimation(
    fig,
    update,
    frames=len(df),
    init_func=init,
    interval=30,
    blit=False
)

plt.show()
