from omni.isaac.debug_draw import _debug_draw
import numpy as np
import subprocess
import sys

# 安装 transforms3d
subprocess.check_call([sys.executable, "-m", "pip", "install", "transforms3d"])
import transforms3d
from transforms3d.quaternions import quat2mat

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

def draw_coordinate_system(position, w, x, y, z, axis_length=0.05, arrow_size=0.01):
    """
    根据给定的四元数和位置画出一个局部坐标系，包含X、Y、Z三个轴。
    
    参数:
        position: 坐标系的原点 (x, y, z)
        quaternion: 四元数 (w, x, y, z)，表示旋转
        axis_length: 坐标轴的长度，默认0.05米
        arrow_size: 箭头的大小，默认0.01米
    """
    # 将四元数转换为旋转矩阵
    rotation_matrix = quat2mat((w,x,y,z))
    
    # 提取旋转矩阵的三个轴方向
    x_axis = rotation_matrix[:, 0]  # X轴方向
    y_axis = rotation_matrix[:, 1]  # Y轴方向
    z_axis = rotation_matrix[:, 2]  # Z轴方向

    # 绘制X轴 (红色)
    draw_line_with_arrow(position, x_axis, axis_length, color=(1.0, 0.0, 0.0, 1.0), arrow_size=arrow_size)

    # 绘制Y轴 (绿色)
    draw_line_with_arrow(position, y_axis, axis_length, color=(0.0, 1.0, 0.0, 1.0), arrow_size=arrow_size)

    # 绘制Z轴 (蓝色)
    draw_line_with_arrow(position, z_axis, axis_length, color=(0.0, 0.0, 1.0, 1.0), arrow_size=arrow_size)

# 使用示例
## 画出tube的局部坐标系
# start = (0.44589391350746155,0.033495668321847916,0.001561759039759636)  # 起点
# direction = ( 0.3049119959512959, 0.9186423864602333, 0.25124657315801197)  # 物体轴线方向向量
# length = 0.05  # 线的长度
# draw_line_with_arrow(start, direction, length, color=(0.0, 1.0, 0.0, 1.0), arrow_size=0.01)

# direction = (0.9490861110659287, -0.3150167515891047, 0.0)  # 水平方向向量
# length = 0.05  # 线的长度
# draw_line_with_arrow(start, direction, length, color=(0.0, 0.0, 1.0, 1.0), arrow_size=0.01)

# direction = (-0.07914687932413128, -0.23845463303717893, 0.9679231165109943)  # 垂轴线的方向向量
# length = 0.05  # 线的长度
# draw_line_with_arrow(start, direction, length, color=(1.0, 0.0, 0.0, 1.0), arrow_size=0.01)

# ## 画出预抓取位置坐标系
# start = ( 0.4300645376426353, -0.014195258285587875, 0.1 )
# direction = (0.07914687932413128, 0.23845463303717893, -0.9679231165109943) # 末端z轴
# length = 0.05  # 线的长度
# draw_line_with_arrow(start, direction, length, color=(0.0, 0.0, 1.0, 1.0), arrow_size=0.01)

# direction = (0.9490861110659287, -0.3150167515891047, 0.0)  # 末端x轴
# length = 0.05  # 线的长度
# draw_line_with_arrow(start, direction, length, color=(1.0, 0.0, 0.0, 1.0), arrow_size=0.01)

# direction = (-0.3049119959512959, -0.9186423864602333, -0.25124657315801197)  # 末端y轴
# length = 0.05  # 线的长度
# draw_line_with_arrow(start, direction, length, color=(0.0, 1.0, 0.0, 1.0), arrow_size=0.01)

# 测试函数，输入四元数和位置，绘制坐标系
position = (0.5006412658518109, 0.04701351999196219, 0.10311319470614495)

# 调用函数绘制坐标系
draw_coordinate_system(position,  w=0.012691609312534352, x=-0.9988892445635018, y=-0.045374748201138335, z=0.0005765189483796254
)

position = (0.48532125626194256, 0.040058163948306205, 0.10309987422880523)

# 调用函数绘制坐标系
draw_coordinate_system(position,  w=0.011969156053142554, x=-0.9886175453711581, y=-0.14996263505652674, z=0.001815592075556542
)

position = (0.4180536507384895, 0.03650110312341828, 0.10307092461432411)

# 调用函数绘制坐标系
draw_coordinate_system(position,  w=0.015474026577352593, x=-0.9955851415424879, y=0.09256732936193976, z=-0.0014387411532899802
)

position = (0.4379792255750484, 0.00965020501813002, 0.09835407069085907)

# 调用函数绘制坐标系
draw_coordinate_system(position,  w=0.12502060639620682, x=-0.9792409833688994, y=0.15826766803806785, z=-0.020206180263166916
)