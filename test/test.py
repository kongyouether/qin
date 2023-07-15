import numpy as np

# 定义坐标点组
points = np.array([(18, 0), (18, 1), (18, 2), (17, 2), (16, 2), (16, 1), (16, 0), (15, 0), (14, 0), (14, 1), (14, 2), (13, 2), (12, 2), (12, 3), (12, 4), (12, 5), (12, 6), (13, 6), (14, 6), (15, 6), (16, 6), (16, 7), (16, 8), (17, 8), (18, 8), (18, 7), (18, 6), (18, 5), (18, 4), (17, 4), (16, 4), (15, 4), (14, 4)])

# 计算拐点
corners = []
print(len(points) - 1)
for i in range(1,len(points) - 1,1):
    dx = points[i+1][0] - points[i-1][0]
    dy = points[i+1][1] - points[i-1][1]
    delta = abs(dx * dx) + abs(dy * dy)
    if delta == 2:
        corners.append(points[i])
    else:
        continue

# 输出拐点坐标
for corner in corners:
    print(corner)