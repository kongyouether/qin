import numpy as np
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import random


def get_distance(x1, y1, x2, y2):
    return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def get_total_distance(order, distance_matrix):
    total_distance = 0
    num_cities = len(order)
    for i in range(num_cities):
        j = (i + 1) % num_cities
        city_i = order[i]
        city_j = order[j]
        total_distance += distance_matrix[city_i][city_j]
    return total_distance


def get_neighbor(order):
    num_cities = len(order)
    i = random.randint(0, num_cities - 1)
    j = random.randint(0, num_cities - 1)
    neighbor = order.copy()
    neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
    return neighbor


def simulated_annealing(distance_matrix, initial_temperature=1000, cooling_rate=0.99):
    # 获取城市数量和城市顺序，利用乱序算法得到初始的城市顺序。
    num_cities = len(distance_matrix)
    order = list(range(num_cities))
    random.shuffle(order)
    # 设置当前城市顺序和当前距离为初始值，同时将当前城市顺序和距离作为最优城市顺序和距离。
    current_order = order.copy()
    current_distance = get_total_distance(current_order, distance_matrix)
    best_order = current_order.copy()
    best_distance = current_distance
    temperature = initial_temperature
    plt.ion()
    fig, ax = plt.subplots()
    import cv2

    # 定义视频保存的参数
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    video = cv2.VideoWriter('output.avi', fourcc, 20, (800, 600))
    while temperature > 1:
        # 随机交换两个城市的顺序，得到邻近的城市顺序。
        neighbor_order = get_neighbor(current_order)
        # 计算当前距离和邻近距离的差值。
        neighbor_distance = get_total_distance(neighbor_order, distance_matrix)
        delta_distance = neighbor_distance - current_distance
        # 如果新城市顺序的距离比当前顺序的距离小，则用新顺序替换当前顺序，
        # 并更新最优顺序和最优距离
        if delta_distance < 0:
            current_order = neighbor_order.copy()
            current_distance = neighbor_distance
            if current_distance < best_distance:
                best_order = current_order.copy()
                best_distance = current_distance
        else:
            # 如果新城市顺序的距离比当前顺序的距离大，则以一定的概率接受新顺序，
            acceptance_probability = np.exp(-delta_distance / temperature)
            if random.random() < acceptance_probability:
                current_order = neighbor_order.copy()
                current_distance = neighbor_distance
        temperature *= cooling_rate
        # 绘制结果图像
        ax.scatter(x_coords, y_coords)
        for i in range(num_cities):
            j = (i + 1) % num_cities
            city_i = current_order[i]
            city_j = current_order[j]
            ax.plot([x_coords[city_i], x_coords[city_j]], [y_coords[city_i], y_coords[city_j]], 'r')
        fig.canvas.draw()
        fig.canvas.flush_events()
        # 将绘制图片存入视频中
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img.shape = (600, 800, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        video.write(img)
        ax.clear()
    # 释放视频对象
    video.release()
    return best_order


# 测试代码
if __name__ == '__main__':
    # 城市坐标数据，这里使用随机生成的数据
    num_cities = 20
    x_coords = np.random.rand(num_cities) * 1000
    y_coords = np.random.rand(num_cities) * 1000
    # 计算距离矩阵
    distance_matrix = np.zeros((num_cities, num_cities))
    for i in range(num_cities):
        for j in range(i + 1, num_cities):
            distance_matrix[i][j] = get_distance(x_coords[i], y_coords[i], x_coords[j], y_coords[j])
            distance_matrix[j][i] = distance_matrix[i][j]
    # 模拟退火算法求解旅行商问题
    best_order = simulated_annealing(distance_matrix)

