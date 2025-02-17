import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
import time
from VSSP import VSSP

# センサの設定
sensor = VSSP()
IPAddress = "192.168.0.10"
port = 10940

# IPアドレスの指定がされていればそのIPアドレスに接続
if len(sys.argv) >= 2:
    IPAddress = sys.argv[1]
print("connect to " + IPAddress)
sensor.open(IPAddress, port)

# 水平インターレース4, 垂直インターレース 1 に設定
sensor.InitialSetting(horizontalInterlace = 4, verticalInterlace = 1)
# 計測開始(強度無し)
sensor.StartMeasurement(isEnableIntensity = False)

lastFrame = -1
frameX = []
frameY = []
frameZ = []

# 3Dプロットの設定
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
# 軸ラベルを設定
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title('3D Scatter Plot')
# 軸の範囲を設定
ax.set_xlim([-2000, 2000])
ax.set_ylim([-2000, 2000])
ax.set_zlim([-2000, 2000])

sc = ax.scatter([], [], [])

def updateData(frame):
    global lastFrame
    global frameX
    global frameY
    global frameZ
    while(True):
        # 計測データの取得
        tmp = sensor.ReceiveMeasurementData()
        if (tmp):
            (measurementHeader, polarPoints, intensities) = tmp
            # フレームが変わったタイミングで描画
            if (measurementHeader.frame != lastFrame):
                lastFrame = measurementHeader.frame
                sc._offsets3d = (frameX, frameY, frameZ)
                frameX = []
                frameY = []
                frameZ = []
                return sc,

            # フレームが変わるまでx,y,zを追加する
            # 極座標を直交座標に変換
            XYZPoints = polarPoints.ToXYZPoints()
            frameX += XYZPoints.x
            frameY += XYZPoints.y
            frameZ += XYZPoints.z

# 50ms間隔で更新する
ani = FuncAnimation(fig, updateData, interval = 50, cache_frame_data = False)
plt.show()
