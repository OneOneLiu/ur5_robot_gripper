import numpy as np
import transforms3d
from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.axangles import axangle2mat

current_tube_rotation_matrix = np.array([[1.0, 0.0, 0.0],
                                            [0.0, 1.0, 0.0],
                                            [0.0, 0.0, 1.0]])

current_y_axis = current_tube_rotation_matrix[:, 1]  # 当前的 y 轴方向
target_y_axis = np.array([0, 0, 1])  # 目标 y 轴是世界坐标系中的 z 轴

# 计算将当前 y 轴旋转到目标 y 轴的旋转矩阵
# 先计算旋转轴
rotation_axis = np.cross(current_y_axis, target_y_axis)
if np.linalg.norm(rotation_axis) > 1e-6:  # 避免零向量的情况
    rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
    # 再计算旋转角度
    rotation_angle = np.arccos(np.dot(current_y_axis, target_y_axis))
    rotation_matrix_to_target_y = axangle2mat(rotation_axis, rotation_angle)
    quterion = mat2quat(rotation_matrix_to_target_y)
else:
    # 如果 y 轴已经对齐，不需要旋转
    rotation_matrix_to_target_y = np.eye(3)

tube_y_2_world_z_rotation_matrix = np.dot(rotation_matrix_to_target_y, current_tube_rotation_matrix)
y_2_w_quterion = mat2quat(tube_y_2_world_z_rotation_matrix)

candidate_quaternions = []

## 下面需要生成一组候选的，具有不同的x,z轴旋转的旋转矩阵，也就是给它乘上不同的绕世界z的旋转就行了
for angle_deg in range(0, 360, 10):
    angle_rad = np.radians(angle_deg)
    rotation_matrix = axangle2mat([0, 0, 1], angle_rad)
    final_rotation_matrix = np.dot(rotation_matrix, tube_y_2_world_z_rotation_matrix) # np.dot(R1, R2)：这表示先执行 R2 的旋转，然后执行 R1 的旋转。
    final_quaternion = mat2quat(final_rotation_matrix)
    candidate_quaternions.append(final_quaternion)
pass