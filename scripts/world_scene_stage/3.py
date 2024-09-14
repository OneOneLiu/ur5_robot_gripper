#!/bin/bash python3

## 获取场景中物体的位置
from omni.isaac.core import World

## 第二次运行时注释掉下面两行
if World.instance():
    world.clear_instance()

world = World()

world.scene.add_default_ground_plane()

## 2中已经说过，可以通过多种方式创建isaac sim的物体, 这是因为isaac sim本就是一个多层架构的复杂软件平台，不同的软件层都可以实现类似的功能，这一点在获取场景信息上也有体现，比如下面我们创建一个cuboid物体，并通过两种方式从isaac sim中获取当前的stage，他们的效果是一样的

## 第二次运行时注释掉下面两行
dy_cube = DynamicCuboid(prim_path = "/World/cube", name="cuboid", scale=np.array([1, 1, 1]), position=np.array([0.0, 0.0, 1.0]), color=np.array([0, 0, 1.0]), size = 1.0)
world.scene.add(dy_cube)

stage = world.scene.stage
cube_prim = stage.GetPrimAtPath("/World/cube")
print(cube_prim)

import omni
usd_context = omni.usd.get_context()
stage = usd_context.get_stage()

cube_prim = stage.GetPrimAtPath("/World/cube")

print(cube_prim)

## 下面我们同样用两种方式获取物体的位置信息，但是他们的效果略有差异

### 第一种方式
timeline = omni.timeline.get_timeline_interface()
timecode = timeline.get_current_time() * timeline.get_time_codes_per_seconds()
pose = omni.usd.get_world_transform_matrix(cube_prim, timecode) # 获取物体的世界坐标系的位置
# print("Matrix Form:", pose)
print("Translation: ", pose.ExtractTranslation())
q = pose.ExtractRotation().GetQuaternion()
print(
    "Rotation: ", q.GetReal(), ",", q.GetImaginary()[0], ",", q.GetImaginary()[1], ",", q.GetImaginary()[2]
)

## 第二种方式
from omni.isaac.dynamic_control import _dynamic_control

dc=_dynamic_control.acquire_dynamic_control_interface()

object=dc.get_rigid_body("/World/cube")
object_pose=dc.get_rigid_body_pose(object)

print("path:/World/cube")
print("position:", object_pose.p)
print("rotation:", object_pose.r)

# 不要启动仿真，运行上述代码，可以看到输出如下：
'''
Usd.Prim(</World/cube>)
Usd.Prim(</World/cube>)
# 第一种方式的输出
Translation:  (0, 0, 1)
Rotation:  1.0 , 0.0 , 0.0 , 0.0


# 第二种方式的输出
2024-09-14 00:15:48 [11,096,479ms] [Error] [omni.isaac.dynamic_control.plugin] Failed to register rigid body at '/World/cube'
2024-09-14 00:15:48 [11,096,479ms] [Warning] [omni.isaac.dynamic_control.plugin] DcGetRigidBodyPose: Function called while not simulating
path:/World/cube
position: (0,0,0)
rotation: (0,0,0,1)
'''

'''从输出中我们解读如下信息：
1. 第一种方式通过获取物体transform的方式获取物体信息，是基于USD的方法，正常情况下，我们去读取物体的transform属性标签，他是相对于其父Xform的，不是世界坐标系的，但在这里我们使用的是get_world_transform_matrix函数，可以直接返回相对于世界坐标系的变换，也就是物体位置
2. 第一种方式成功了，因为基于USD的方法是对场景的静态描述，不需要仿真也可读取

3. 第二种方式通过动力学控制的方式获取物体的位置信息，但是失败了，因为我们没有启动仿真，动力学控制是基于仿真的，所以在没有仿真的情况下，是无法获取物体的位置信息的
4. 这种方式获取到的信息直接就会是刚体的位置信息，是相对于世界坐标系的，不需要额外的转换'''

## 在GUI中启动仿真，注释掉上述代码，然后再次运行上面的代码，可以看到输出如下：

'''
Usd.Prim(</World/cube>)
Usd.Prim(</World/cube>)
# 第一种方式的输出
Translation:  (-0.000026570742193143815, 0.000028746830139425583, 0.5000000596046448)
Rotation:  0.9999999999997464 , -6.998202781918227e-07 , 2.8111172778546427e-08 , -1.2888451359741367e-07
path:/World/cube
# 第二种方式的输出
position: (-2.65707e-05,2.87468e-05,0.5)
rotation: (-6.99741e-07,2.8108e-08,-1.2887e-07,1) 
'''

'''
从输出中我们解读如下信息：
1. 两种方式都成功了，因为我们启动了仿真，动力学控制的方式获取物体的位置信息，是基于仿真的，所以在仿真的情况下，是可以获取物体的位置信息的
2. 两者的位置信息是一样的，只是精度不同
3. 两者的旋转信息是一样的，都是四元数，但是数值顺序不同，第一种是 w, x, y, z, 第二种是 x, y, z, w。
'''

'''其他说明
1. 这里获取到的位置信息，都是物体的原点位置信息，对于在isaac sim中创建的物体，其原点位置信息是相对于物体网格的几何中心的（经过测试得到）
2. 在GUI点击物体会看到显示一个以物体为中心的坐标系，这个坐标系的原点并不一定是上述表示物体位置的原点，只是为了给物体施加移动，旋转，缩放等变换时所用的参考坐标系，英文称为pivot(枢轴)。枢轴的位置和导向可以调节，但是物体的原点位置是不变的，这个原点位置是我们上述获取的位置信息。
3. 在自定义的模型中，比如自己用blender创建的物体时，即使设置了原点，在使用obj, stl等格式导入时，isaac sim会将其原点自动设置为物体的几何中心，或者bounding box的一角 （目前还没发现规律）。但是从blender中将模型导出为usd格式时，原点位置是可以保留的，这时我们获取到的位置信息就是相对于原点的位置信息了。
'''