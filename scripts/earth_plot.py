import numpy as np
import pandas as pd
import plotly.graph_objects as go

# ===== 1. 读数据 =====
df = pd.read_csv("../runs/baseline_mc_001/instance_0000/trajectory.csv")


mx, my, mz = df["missile_x_ecef_m"], df["missile_y_ecef_m"], df["missile_z_ecef_m"]
tx, ty, tz = df["target_x_ecef_m"], df["target_y_ecef_m"], df["target_z_ecef_m"]

# ===== 2. 生成地球球面 =====
R = 6371000  # 地球半径（米）

u = np.linspace(0, 2*np.pi, 80)
v = np.linspace(0, np.pi, 40)

x = R * np.outer(np.cos(u), np.sin(v))
y = R * np.outer(np.sin(u), np.sin(v))
z = R * np.outer(np.ones_like(u), np.cos(v))

earth = go.Surface(
    x=x,
    y=y,
    z=z,
    colorscale="Blues",
    opacity=0.4,
    showscale=False,
    name="Earth"
)

# ===== 3. 导弹轨迹 =====
missile = go.Scatter3d(
    x=mx,
    y=my,
    z=mz,
    mode="lines",
    name="Missile",
    line=dict(color="red", width=4)
)

# ===== 4. 目标轨迹 =====
target = go.Scatter3d(
    x=tx,
    y=ty,
    z=tz,
    mode="lines",
    name="Target",
    line=dict(color="blue", width=4)
)

# ===== 5. 当前点 =====
missile_pt = go.Scatter3d(
    x=[mx.iloc[-1]],
    y=[my.iloc[-1]],
    z=[mz.iloc[-1]],
    mode="markers",
    marker=dict(size=5, color="red"),
    name="Missile current"
)

target_pt = go.Scatter3d(
    x=[tx.iloc[-1]],
    y=[ty.iloc[-1]],
    z=[tz.iloc[-1]],
    mode="markers",
    marker=dict(size=5, color="blue"),
    name="Target current"
)

# ===== 6. layout =====
layout = go.Layout(
    title="Missile Guidance on Earth (ECEF)",
    scene=dict(
        xaxis_title="ECEF X (m)",
        yaxis_title="ECEF Y (m)",
        zaxis_title="ECEF Z (m)",
        aspectmode="data"  # 🔥关键：防止球变形
    )
)

# ===== 7. plot =====
fig = go.Figure(data=[earth, missile, target, missile_pt, target_pt], layout=layout)

fig.show()
