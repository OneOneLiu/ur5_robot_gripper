from omni.isaac.debug_draw import _debug_draw
import numpy as np

# 获取 debug_draw 接口
draw = _debug_draw.acquire_debug_draw_interface()

def draw_line_with_arrow(start_point, direction, length, color=(1.0, 0.0, 0.0, 1.0), size=5, arrow_size=0.1):
    """
    画一条带箭头的线段，给定起点、方向和长度。
    
    参数:
        start_point: 线的起点坐标 (x, y, z)
        direction: 线的方向向量 (dx, dy, dz)，应该是单位向量
        length: 线的长度
        color: 线的颜色 (r, g, b, a)，默认红色
        size: 线的粗细，默认值为 5
        arrow_size: 箭头的大小，默认值为 0.1
    """
    # 将方向向量归一化
    direction = np.array(direction)
    direction_normalized = direction / np.linalg.norm(direction)
    
    # 计算终点位置
    end_point = np.array(start_point) + direction_normalized * length
    
    # 画线
    draw.draw_lines(
        [start_point],  # 起点列表
        [end_point],    # 终点列表
        [color],        # 颜色列表
        [size]          # 线的粗细列表
    )
    
    # 箭头的左右两边
    left_wing = np.array([-direction_normalized[1], direction_normalized[0], 0])  # 假设Z轴平面
    right_wing = np.array([direction_normalized[1], -direction_normalized[0], 0])  # 假设Z轴平面
    
    # 左侧箭头线的终点
    left_arrow_point = end_point - direction_normalized * arrow_size + left_wing * arrow_size
    # 右侧箭头线的终点
    right_arrow_point = end_point - direction_normalized * arrow_size + right_wing * arrow_size
    
    # 画箭头的两个边
    draw.draw_lines(
        [end_point, end_point],  # 起点是线的终点
        [left_arrow_point, right_arrow_point],  # 箭头两侧的终点
        [color, color],  # 颜色列表
        [size, size]  # 线的粗细列表
    )

# 使用示例
start = (  0.48903465270996094,0.16054636240005493,0.006851098500192165)  # 起点
direction = ( -0.009354244011200441, -0.9994058682640286, -0.033172407184313135)  # 方向向量
length = 0.1  # 线的长度
draw_line_with_arrow(start, direction, length, color=(1.0, 0.0, 0.0, 1.0), arrow_size=0.01)

start = (0.48903465270996094,0.16054636240005493,0.006851098500192165)  # 起点
direction = ( -0.9999561999033177, 0.009359395008012113, 0.0 )  # 方向向量
length = 0.1  # 线的长度
draw_line_with_arrow(start, direction, length, color=(0.0, 1.0, 0.0, 1.0), arrow_size=0.01)

start = (0.48903465270996094,0.16054636240005493,0.006851098500192165)  # 起点
direction = (-0.00031047366220460557, -0.033170954229671284, 0.9994496442550761)  # 方向向量
length = 0.1  # 线的长度
draw_line_with_arrow(start, direction, length, color=(0.0, 0.0, 1.0, 1.0), arrow_size=0.01)