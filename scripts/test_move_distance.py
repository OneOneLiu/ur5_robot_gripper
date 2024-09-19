import itertools
import math

# 机器人初始位置
robot_initial_pos = (0.5, 0.0, 0.4)

# 试管的位置和姿态（您提供的数据）
tube_positions_input = [
    {
        'position': {'x': 0.6252490282058716, 'y': 0.0470949225127697, 'z': -0.005291961133480072},
        'orientation': {'x': 0.03626063093543053, 'y': 0.4668007791042328, 'z': 0.10170304775238037, 'w': -0.8777464032173157}
    },
    {
        'position': {'x': 0.5234113931655884, 'y': 0.04707350954413414, 'z': -0.005295674316585064},
        'orientation': {'x': 0.04498918354511261, 'y': -0.10829377919435501, 'z': -0.49993184208869934, 'w': -0.8580888509750366}
    },
    {
        'position': {'x': 0.5395268797874451, 'y': -0.08335358649492264, 'z': -0.005292756017297506},
        'orientation': {'x': -0.08563557267189026, 'y': -0.07730121910572052, 'z': 0.6407251358032227, 'w': -0.7590537071228027}
    },
    {
        'position': {'x': 0.5353776812553406, 'y': 0.20550809800624847, 'z': -0.005294648930430412},
        'orientation': {'x': -0.5820416808128357, 'y': 0.426369309425354, 'z': 0.5712339282035828, 'w': 0.39131635427474976}
    },
    {
        'position': {'x': 0.4317117929458618, 'y': 0.11456650495529175, 'z': -0.0052948761731386185},
        'orientation': {'x': -0.8364906311035156, 'y': 0.16153478622436523, 'z': 0.517362117767334, 'w': 0.08078619092702866}
    }
]

# 放置位置列表，根据试管数量生成
placement_positions = [(0.6, 0.32 + 0.016 * i, 0.07) for i in range(5)]

def euclidean_distance(point1, point2):
    """计算两个三维点之间的欧几里得距离。"""
    return math.sqrt(sum((p2 - p1) ** 2 for p1, p2 in zip(point1, point2)))

def calculate_total_distance(tube_positions, permutation):
    """计算给定试管排列下的总移动距离。"""
    total_distance = 0.0

    # 从机器人初始位置到第一个试管的位置
    total_distance += euclidean_distance(robot_initial_pos, tube_positions[permutation[0]])

    for i in range(len(permutation)):
        # 从试管到放置位置
        total_distance += euclidean_distance(tube_positions[permutation[i]], placement_positions[i])

        if i < len(permutation) - 1:
            # 从放置位置到下一个试管的位置
            total_distance += euclidean_distance(placement_positions[i], tube_positions[permutation[i + 1]])

    return total_distance

def main():
    # 提取试管的位置数据
    tube_positions = []
    for tube in tube_positions_input:
        pos = tube['position']
        tube_positions.append((pos['x'], pos['y'], pos['z']))

    # 试管的索引列表
    tube_indices = list(range(len(tube_positions)))

    # 生成所有可能的试管排列
    permutations = list(itertools.permutations(tube_indices))

    # 存储每个排列的总距离
    total_distances = []

    print("正在计算，请稍候...")

    # 遍历所有排列并计算总距离
    for perm in permutations:
        total_distance = calculate_total_distance(tube_positions, perm)
        total_distances.append({'permutation': perm, 'total_distance': total_distance})

    # 按总距离排序
    total_distances.sort(key=lambda x: x['total_distance'])

    # 输出结果
    print("每种排列的总移动距离（按距离从小到大排序）：")
    for result in total_distances:
        perm = result['permutation']
        distance = result['total_distance']
        perm_tubes = [f"Tube{idx + 1}" for idx in perm]
        print(f"排列 {perm_tubes}: 总距离 = {distance:.4f} 米")

    # 输出最优排列
    best_result = total_distances[0]
    best_perm = best_result['permutation']
    best_distance = best_result['total_distance']
    best_perm_tubes = [f"Tube{idx + 1}" for idx in best_perm]
    print("\n最优排列：")
    print(f"排列 {best_perm_tubes}: 总距离 = {best_distance:.4f} 米")

if __name__ == "__main__":
    main()
