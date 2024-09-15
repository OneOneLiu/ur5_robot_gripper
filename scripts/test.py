from omni.isaac.core.utils.stage import add_reference_to_stage
import omni
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

add_reference_to_stage("/catkin_ws/src/ur5_robot_gripper/meshes/tube75.usd", "/World/ur5_robot_gripper")

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

set_object_transforms("/World/ur5_robot_gripper", (0.5, 0.05, 0.05), (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))