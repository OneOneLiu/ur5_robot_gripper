from omni.isaac.core import World
import numpy as np

# Hello world: https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html

# Glossary: https://docs.omniverse.nvidia.com/isaacsim/latest/reference_glossary.html#isaac-sim-glossary-stage

# API REFERENCE: https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html

world = World.instance()
print(world)

world = World()
print(world)

world = World.instance()
print(world)

print(world.play())

## 启动GUI窗口后是没有自动实例化world对象的
## 直接通过World.instance()获取World对象是空的
## 通过World()实例化World对象，然后再通过World.instance()获取World对象，这样就可以获取到World对象了

# 输出1：
# None
# <omni.isaac.core.world.world.World object at 0x7fdf65968a60>
# <omni.isaac.core.world.world.World object at 0x7fdf65968a60>

## 而且即使通过GUI新建一个窗口，或者打开一个USD，World对象仍然是它，不会销毁或者新建World对象
# 输出2：
# <omni.isaac.core.world.world.World object at 0x7fdf65968a60>
# <omni.isaac.core.world.world.World object at 0x7fdf65968a60>
# <omni.isaac.core.world.world.World object at 0x7fdf65968a60>

## World对象是Singleton模式，即只有一个实例，在运行isaac sim的时候，只有一个World对象可以存在，但是它可以通过下面的命令销毁
world.clear_instance() # 取消注释此行，则程序的输出一直都会是上面的输出1 https://docs.omniverse.nvidia.com/py/isaacsim/source/extensions/omni.isaac.core/docs/index.html#module-omni.isaac.core.world
