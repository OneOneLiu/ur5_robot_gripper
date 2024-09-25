from omni.isaac.core.utils.stage import add_reference_to_stage
import omni
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

def set_object_transforms(object_prim_path, translation, rotation, scale):
        '''Set the object transforms
        Args:
            object_prim_path (str): The path to the object prim
            translation (list): The translation values (mm)
            rotation (list): The rotation values (degree)
            scale (list): The scale values
        '''
        # TODO: Check if this is the elegant way to set the object transforms, if not, refactor the code. Becuase this method seems to transform the object into an Xformable object
        object_prim = stage.GetPrimAtPath(object_prim_path)
        xformable = UsdGeom.Xformable(object_prim)
        xformable.SetXformOpOrder([])
        translateop = xformable.AddTranslateOp()
        translateop.Set(Gf.Vec3d(translation))

        xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(rotation))

        xformable.AddScaleOp().Set(Gf.Vec3f(scale))

add_reference_to_stage("/catkin_ws/src/ur5_robot_gripper/meshes/tray/tray.usd", "/World/tray")
set_object_transforms("/World/tray", (0.5, 0.05, 0.0), (90.0, 0.0, 0.0), (1, 1, 1))


for i in range(5):
    add_reference_to_stage("/catkin_ws/src/ur5_robot_gripper/meshes/tube75/tube75.usd", "/World/tube75_{}".format(i))
    set_object_transforms("/World/tube75_{}".format(i), (0.5, 0.05, 0.05+0.02*i), (0.0, 0.0, 0.0), (0.001, 0.001, 0.001))

for i in range(2):
    add_reference_to_stage("/catkin_ws/src/ur5_robot_gripper/meshes/rack/rack.usd", "/World/rack{}".format(i))
    set_object_transforms("/World/rack{}".format(i), (0.5, 0.5, 0.05+0.2*i), (0.0, 0.0, 0.0), (0.001, 0.001, 0.001))