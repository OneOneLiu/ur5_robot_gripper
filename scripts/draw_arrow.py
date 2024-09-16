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
## 画出tube的局部坐标系
start = (0.44589391350746155,0.033495668321847916,0.001561759039759636)  # 起点
direction = ( 0.3049119959512959, 0.9186423864602333, 0.25124657315801197)  # 物体轴线方向向量
length = 0.05  # 线的长度
draw_line_with_arrow(start, direction, length, color=(0.0, 1.0, 0.0, 1.0), arrow_size=0.01)

direction = (0.9490861110659287, -0.3150167515891047, 0.0)  # 水平方向向量
length = 0.05  # 线的长度
draw_line_with_arrow(start, direction, length, color=(0.0, 0.0, 1.0, 1.0), arrow_size=0.01)

direction = (-0.07914687932413128, -0.23845463303717893, 0.9679231165109943)  # 垂轴线的方向向量
length = 0.05  # 线的长度
draw_line_with_arrow(start, direction, length, color=(1.0, 0.0, 0.0, 1.0), arrow_size=0.01)

## 画出预抓取位置坐标系

start = ( 0.4300645376426353, -0.014195258285587875, 0.1 )
direction = (0.07914687932413128, 0.23845463303717893, -0.9679231165109943) # 末端z轴
length = 0.05  # 线的长度
draw_line_with_arrow(start, direction, length, color=(0.0, 0.0, 1.0, 1.0), arrow_size=0.01)

direction = (0.9490861110659287, -0.3150167515891047, 0.0)  # 末端x轴
length = 0.05  # 线的长度
draw_line_with_arrow(start, direction, length, color=(1.0, 0.0, 0.0, 1.0), arrow_size=0.01)

direction = (-0.3049119959512959, -0.9186423864602333, -0.25124657315801197)  # 末端y轴
length = 0.05  # 线的长度
draw_line_with_arrow(start, direction, length, color=(0.0, 1.0, 0.0, 1.0), arrow_size=0.01)