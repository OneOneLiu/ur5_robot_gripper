#!/usr/bin/env python3
# 理解如何往场景中添加物体与控制物体

from omni.isaac.core import World
if World.instance():
    world.clear_instance()
world = World()
world.clear()
## World 里面包含一个scene对象，通过这个scene对象可以往场景中添加，删除，获取物体信息等操作。
## https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#scenes

## 添加物体到场景中
## 添加一个默认地面，默认的prim_path是"/World/defaultGroundPlane"
ground_plane = world.scene.add_default_ground_plane(prim_path="/defaultGroundPlane")

## 其他还能添加的物体有：cone, cuboid, cylinder, 等等， 操作流程见：https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#objects

# world.clear()

## TODO: 此程序只能运行一次，否则会报错，因为即使新建窗口，world对象仍然是它，不会清空场景中的物体
## 这似乎是一个bug，因为场景中的物体从GUI看是被清空了，通过代码看也是被清空了，但是再次运行程序时，会报错，说名称已经存在了

## 需要先创建一个物体，然后再使用add方法添加到场景中
## 添加一个dynamic cuboid, 与fixed cuboid的区别是dynamic cuboid具备碰撞属性和刚体动力学属性（rigid body api）
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.objects import DynamicCone
import numpy as np
dy_cube = DynamicCuboid(prim_path = "/World/cube", name="cuboid", scale=np.array([1, 1, 1]), position=np.array([0.0, 0.0, 0.0]), color=np.array([0, 0, 1.0]), size = 1.0)

world.scene.add(dy_cube)

dy_cube = DynamicCone(prim_path = "/World/cone", name="cone", scale=np.array([1, 1, 1]), position=np.array([1.0, 1.0, 0.0]), color=np.array([1.0, 0, 1.0]), radius = 1.0, height = 1.0)

# world.clear()

## 注意，上面我们演示的是通过isaac core里的API来添加具备动力学属性的刚性物体到场景中，添加后的物体直接就可以用来仿真，同时isaac core api也提供了创建没有物理学属性的视觉物体的方法，以及只具备碰撞属性，不具备运动属性的方法（静态物体），详见：https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#objects

## 与此同时，Isaac sim还可以通过omni.kit的方式来创建物体到场景，这种方式创建的物体是没有物理学属性的，但是可以后续操作添加，详见：https://docs.omniverse.nvidia.com/isaacsim/latest/how_to_guides/environment_setup.html#usd-how-tos