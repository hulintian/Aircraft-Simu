import pandas as pd
import numpy as np
import plotly.graph_objects as go

src = input("input")
# ===== 1. 读数据 =====
df = pd.read_csv(src)

mx = df["missile_x_ecef_m"].values
my = df["missile_y_ecef_m"].values
mz = df["missile_z_ecef_m"].values

tx = df["target_x_ecef_m"].values
ty = df["target_y_ecef_m"].values
tz = df["target_z_ecef_m"].values

t = df["time_s"].values

# ===== 2. frame（关键：动画）=====
frames = []
for i in range(len(df)):
    frames.append(
        go.Frame(
            data=[
                # missile trajectory
                go.Scatter3d(
                    x=mx[:i],
                    y=my[:i],
                    z=mz[:i],
                    mode="lines+markers",
                    name="Missile",
                    line=dict(color="red", width=4),
                ),
                # target trajectory
                go.Scatter3d(
                    x=tx[:i],
                    y=ty[:i],
                    z=tz[:i],
                    mode="lines+markers",
                    name="Target",
                    line=dict(color="blue", width=4),
                ),
                # current missile point
                go.Scatter3d(
                    x=[mx[i]],
                    y=[my[i]],
                    z=[mz[i]],
                    mode="markers",
                    marker=dict(size=5, color="red"),
                    name="Missile current",
                ),
                # current target point
                go.Scatter3d(
                    x=[tx[i]],
                    y=[ty[i]],
                    z=[tz[i]],
                    mode="markers",
                    marker=dict(size=5, color="blue"),
                    name="Target current",
                ),
            ],
            name=str(i),
        )
    )

# ===== 3. 初始帧 =====
init = frames[0].data

# ===== 4. layout =====
layout = go.Layout(
    title="Missile Guidance Simulation (ECEF)",
    scene=dict(
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Z (m)",
    ),
    updatemenus=[
        dict(
            type="buttons",
            showactive=False,
            buttons=[
                dict(label="Play",
                     method="animate",
                     args=[None,
                           dict(frame=dict(duration=30, redraw=True),
                                fromcurrent=True)]),
                dict(label="Pause",
                     method="animate",
                     args=[[None],
                           dict(frame=dict(duration=0, redraw=False),
                                mode="immediate")]),
            ],
        )
    ],
    sliders=[
        dict(
            steps=[
                dict(
                    method="animate",
                    args=[[str(k)],
                          dict(mode="immediate",
                               frame=dict(duration=0, redraw=True),
                               transition=dict(duration=0))],
                    label=str(k),
                )
                for k in range(len(df))
            ]
        )
    ],
)

# ===== 5. figure =====
fig = go.Figure(data=init, frames=frames, layout=layout)

fig.show()
