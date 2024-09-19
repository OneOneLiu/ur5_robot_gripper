import itertools
import math
from tqdm import tqdm
from multiprocessing import Pool, cpu_count
import csv

# 机器人初始位置
robot_initial_pos = (0.5, 0.0, 0.4)

# 试管的位置和姿态（从您的数据中提取）
tube_positions_input = [
    # 请将这里的试管数据替换为您最新的20个试管的位置和姿态
    {
        'position': {'x': 0.5252894163131714, 'y': 0.1028594821691513, 'z': -0.005306440871208906},
        'orientation': {'x': 0.13730959594249725, 'y': 0.9751025438308716, 'z': -0.008093765936791897, 'w': 0.1739414483308792}
    },
    {
        'position': {'x': 0.5863626599311829, 'y': -0.1235586553812027, 'z': -0.005286037921905518},
        'orientation': {'x': -0.20242926478385925, 'y': -0.9766059517860413, 'z': -0.0012620865600183606, 'w': -0.07253599911928177}
    },
    {
        'position': {'x': 0.5885522365570068, 'y': 0.15650591254234314, 'z': -0.0053021772764623165},
        'orientation': {'x': 0.9619731307029724, 'y': -0.23825810849666595, 'z': -0.13259565830230713, 'w': -0.016093989834189415}
    },
    {
        'position': {'x': 0.5968919396400452, 'y': 0.06787631660699844, 'z': -0.005294528324157},
        'orientation': {'x': -0.3489423096179962, 'y': -0.3514702022075653, 'z': -0.6340499520301819, 'w': 0.5938758254051208}
    },
    {
        'position': {'x': 0.615209698677063, 'y': 0.1501295268535614, 'z': -0.0052941180765628815},
        'orientation': {'x': -0.9352917075157166, 'y': -0.3236645758152008, 'z': 0.1289917528629303, 'w': -0.06190193444490433}
    },
    {
        'position': {'x': 0.5602245330810547, 'y': 0.021988311782479286, 'z': -0.0052980706095695496},
        'orientation': {'x': 0.19386741518974304, 'y': 0.22458016872406006, 'z': -0.5834507346153259, 'w': 0.7560188174247742}
    },
    {
        'position': {'x': 0.6168817281723022, 'y': -0.0947478711605072, 'z': -0.005304822698235512},
        'orientation': {'x': 0.26671138405799866, 'y': 0.9125628471374512, 'z': -0.07047365605831146, 'w': 0.3018733263015747}
    },
    {
        'position': {'x': 0.46963945031166077, 'y': 0.23675264418125153, 'z': -0.005296945571899414},
        'orientation': {'x': -0.45752036571502686, 'y': -0.3742925822734833, 'z': 0.6070640087127686, 'w': -0.5310871005058289}
    },
    {
        'position': {'x': 0.6520259380340576, 'y': 0.16686895489692688, 'z': -0.005295571405440569},
        'orientation': {'x': -0.30337458848953247, 'y': 0.33140844106674194, 'z': -0.5770047307014465, 'w': -0.6820541024208069}
    },
    {
        'position': {'x': 0.6938208341598511, 'y': 0.09936748445034027, 'z': -0.005152668803930283},
        'orientation': {'x': 0.533880352973938, 'y': -0.0009373353677801788, 'z': 0.8450670838356018, 'w': 0.028853416442871094}
    },
    {
        'position': {'x': 0.46278196573257446, 'y': 0.0560784786939621, 'z': -0.005294723901897669},
        'orientation': {'x': -0.34627848863601685, 'y': -0.5547446012496948, 'z': 0.3799911439418793, 'w': -0.6541837453842163}
    },
    {
        'position': {'x': 0.42860302329063416, 'y': -0.043667443096637726, 'z': -0.005252093076705933},
        'orientation': {'x': 0.014950154349207878, 'y': 0.005498089827597141, 'z': 0.09375830739736557, 'w': 0.9954675436019897}
    },
    {
        'position': {'x': 0.44092029333114624, 'y': 0.05756039172410965, 'z': -0.00529607804492116},
        'orientation': {'x': 0.0801360011100769, 'y': 0.7924138903617859, 'z': 0.08063945174217224, 'w': -0.5992960929870605}
    },
    {
        'position': {'x': 0.6734009981155396, 'y': 0.16772893071174622, 'z': -0.008305229246616364},
        'orientation': {'x': 0.7026779055595398, 'y': 0.5663437247276306, 'z': 0.43037936091423035, 'w': 0.016500024124979973}
    },
    {
        'position': {'x': 0.45131826400756836, 'y': 0.14084918797016144, 'z': -0.005286184139549732},
        'orientation': {'x': 0.4848095774650574, 'y': -0.2941822409629822, 'z': 0.6893940567970276, 'w': 0.4507242739200592}
    }
]

def euclidean_distance(point1, point2):
    """计算两个三维点之间的欧几里得距离。"""
    return math.sqrt(sum((p2 - p1) ** 2 for p1, p2 in zip(point1, point2)))

def calculate_total_distance(tube_positions, permutation, placement_positions):
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

def calculate_distance_wrapper(args):
    """包装函数，用于多进程计算。"""
    perm, tube_positions, placement_positions = args
    total_distance = calculate_total_distance(tube_positions, perm, placement_positions)
    return {'permutation': perm, 'total_distance': total_distance}

def save_results_to_csv(n, total_distances, best_result, worst_result, distance_difference, difference_percentage):
    """将结果保存到CSV文件中，包括差值和百分比。"""
    filename = f'tube_{n}_permutations.csv'
    with open(filename, 'w', newline='') as csvfile:
        fieldnames = ['permutation', 'total_distance', 'is_best', 'is_worst', 'distance_difference', 'difference_percentage']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for result in total_distances:
            perm = result['permutation']
            distance = result['total_distance']
            perm_tubes = [f"Tube{idx + 1}" for idx in perm]
            writer.writerow({
                'permutation': str(perm_tubes),
                'total_distance': f"{distance:.4f}",
                'is_best': 'Yes' if perm == list(best_result['permutation']) else '',
                'is_worst': 'Yes' if perm == list(worst_result['permutation']) else '',
                'distance_difference': f"{distance_difference:.4f}" if perm == list(best_result['permutation']) else '',
                'difference_percentage': f"{difference_percentage:.2f}%" if perm == list(best_result['permutation']) else ''
            })

def main():
    # 提取试管的位置数据
    tube_positions = []
    for tube in tube_positions_input:
        pos = tube['position']
        tube_positions.append((pos['x'], pos['y'], pos['z']))

    total_tubes = len(tube_positions)
    
    # 逐步增加试管数量，从2到10（限制为10，避免计算资源过载）
    max_n = min(10, total_tubes)
    for n in range(2, max_n + 1):
        print(f"\n=== 计算前 {n} 个试管的所有排列及其总移动距离 ===")

        # 当前子集的试管位置
        current_tube_positions = tube_positions[:n]

        # 生成对应的放置位置列表
        placement_positions = [(0.6, 0.32 + 0.016 * i, 0.07) for i in range(n)]

        # 试管的索引列表
        tube_indices = list(range(n))

        # 生成所有可能的试管排列
        permutations = list(itertools.permutations(tube_indices))

        # 存储每个排列的总距离
        total_distances = []

        print(f"正在计算 {len(permutations)} 种排列，请稍候...")

        # 使用多进程加快计算速度
        with Pool(processes=cpu_count()) as pool:
            args = [(perm, current_tube_positions, placement_positions) for perm in permutations]
            for result in tqdm(pool.imap(calculate_distance_wrapper, args), total=len(permutations), desc=f"计算前 {n} 个试管的排列"):
                total_distances.append(result)

        # 按总距离排序
        total_distances.sort(key=lambda x: x['total_distance'])

        # 计算最优和最差排列
        best_result = total_distances[0]
        worst_result = total_distances[-1]
        distance_difference = worst_result['total_distance'] - best_result['total_distance']
        difference_percentage = (distance_difference / worst_result['total_distance']) * 100 if worst_result['total_distance'] != 0 else 0

        # 保存结果到CSV文件
        save_results_to_csv(n, total_distances, best_result, worst_result, distance_difference, difference_percentage)

        # 输出结果概要
        print(f"\n前 {n} 个试管的所有排列及其总移动距离已保存到 'tube_{n}_permutations.csv'。")

        # 输出最优排列
        best_perm = best_result['permutation']
        best_distance = best_result['total_distance']
        best_perm_tubes = [f"Tube{idx + 1}" for idx in best_perm]
        print(f"前 {n} 个试管的最优排列：")
        print(f"排列 {best_perm_tubes}: 总距离 = {best_distance:.4f} 米")

        # 输出最差排列
        worst_perm = worst_result['permutation']
        worst_distance = worst_result['total_distance']
        worst_perm_tubes = [f"Tube{idx + 1}" for idx in worst_perm]
        print(f"前 {n} 个试管的最差排列：")
        print(f"排列 {worst_perm_tubes}: 总距离 = {worst_distance:.4f} 米")

        # 输出差值和百分比
        print(f"最优与最差排列的距离差值：{distance_difference:.4f} 米")
        print(f"差值占最差排列距离的百分比：{difference_percentage:.2f}%")

    print("\n所有计算完成。结果已保存到对应的CSV文件中。")

if __name__ == "__main__":
    main()
