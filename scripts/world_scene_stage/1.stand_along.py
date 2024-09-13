import isaacsim
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": True}) # we can also run as headless.

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

world = World.instance()
print(world)

world = World()
print(world)

world = World.instance()
print(world)

## 直接通过World.instance()获取World对象是空的，因为它还没有进行实例化
## 通过World()实例化World对象，然后再通过World.instance()获取World对象，这样就可以获取到World对象了
