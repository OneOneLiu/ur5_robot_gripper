from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from omni.importer.urdf import _urdf
from omni.isaac.franka.controllers import RMPFlowController
from omni.isaac.franka.tasks import FollowTarget
import omni.kit.commands
import omni.usd
# Mimic joint
from pxr import Usd, UsdPhysics, PhysxSchema, Sdf, Gf, UsdGeom

room_asset_path = "/catkin_ws/src/ur5_robot_gripper/meshes/simple_room/simple_room.usd"
omni.usd.get_context().open_stage(room_asset_path)
# Import URDF Robot
# https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_advanced_import_urdf.html#importing-urdf-using-python
urdf_interface = _urdf.acquire_urdf_interface()
# Set the settings in the import config
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.fix_base = True
import_config.make_default_prim = True
import_config.self_collision = False
import_config.create_physics_scene = True
import_config.import_inertia_tensor = False
import_config.default_drive_strength = 1047.19751
import_config.default_position_drive_damping = 52.35988
import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
import_config.distance_scale = 1
import_config.density = 0.0
# Get the urdf file path
extension_path = get_extension_path_from_name("omni.importer.urdf")
# root_path = extension_path + "/data/urdf/robots/franka_description/robots"
root_path = "/catkin_ws/src/ur5_robot_gripper/urdf/ur_description"
# file_name = "panda_arm_hand.urdf"
file_name = "ur5.urdf"
# Finally import the robot
result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path="{}/{}".format(root_path, file_name),
                                                      import_config=import_config,)

stage = omni.usd.get_context().get_stage()

print(prim_path)

# set mimic joints
## /isaac-sim/extsPhysics/omni.physx.demos/omni/physxdemos/scenes/MimicJointDemo.py
# 设置关节的路径，假设这些路径在你的URDF文件中是已知的
left_knuckle_joint_path = prim_path + "/robotiq_85_base_link/robotiq_85_left_knuckle_joint"
left_inner_knuckle_joint_path = prim_path + "/robotiq_85_base_link/robotiq_85_left_inner_knuckle_joint"
print(left_knuckle_joint_path)
print(left_inner_knuckle_joint_path)
left_knuckle_joint_prim = stage.GetPrimAtPath(left_knuckle_joint_path)
left_inner_knuckle_joint_prim = stage.GetPrimAtPath(left_inner_knuckle_joint_path)
# 获取这些关节的Prim
# 应用 mimic API 到左内侧关节
mimic_api = PhysxSchema.PhysxMimicJointAPI.Apply(left_inner_knuckle_joint_prim.GetPrim(), UsdPhysics.Tokens.rotZ)

# 设置 mimic 目标关节
mimic_api.GetReferenceJointRel().AddTarget(left_knuckle_joint_path)

# 设置齿轮比率和偏移（根据实际需求调整）
mimic_api.GetGearingAttr().Set(-1.0)  # 1:1的齿轮比率
mimic_api.GetOffsetAttr().Set(0.0)  # 无偏移

left_finger_tip_joint_path = prim_path + "/robotiq_85_left_inner_knuckle_link/robotiq_85_left_finger_tip_joint"
left_knuckle_joint_prim = stage.GetPrimAtPath(left_knuckle_joint_path)
left_finger_tip_joint_prim = stage.GetPrimAtPath(left_finger_tip_joint_path)
# 获取这些关节的Prim
# 应用 mimic API 到左内侧关节
mimic_api = PhysxSchema.PhysxMimicJointAPI.Apply(left_finger_tip_joint_prim.GetPrim(), UsdPhysics.Tokens.rotZ)

# 设置 mimic 目标关节
mimic_api.GetReferenceJointRel().AddTarget(left_knuckle_joint_path)

# 设置齿轮比率和偏移（根据实际需求调整）
mimic_api.GetGearingAttr().Set(1.0)  # 1:1的齿轮比率
mimic_api.GetOffsetAttr().Set(0.0)  # 无偏移

# 访问assets载入桌子
# https://forums.developer.nvidia.com/t/creating-moving-objects-using-python-scripts/280418
from omni.isaac.core.utils.nucleus import get_assets_root_path
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()
print(assets_root_path)
from omni.isaac.core.utils.stage import add_reference_to_stage
tube75_asset_path = "/catkin_ws/src/ur5_robot_gripper/meshes/tube75/tube75.usd"
for i in range (7):
    for j in range(2):
        add_reference_to_stage(usd_path=tube75_asset_path, prim_path="/World/tube75/tube75_{}_{}".format(i,j))
        object_prim = stage.GetPrimAtPath("/World/tube75/tube75_{}_{}".format(i,j))
        # 添加平移：https://forums.developer.nvidia.com/t/add-and-transform-usd-assets-in-python/250701/4
        xformable = UsdGeom.Xformable(object_prim)
        xformable.SetXformOpOrder([])
        translateop = xformable.AddTranslateOp()
        translateop.Set(Gf.Vec3d(0.5 +0.1*j, 0.05+0.025*j, 0.05+0.035*i))
        if j:
            xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(180.0, 0.0, 0.0))

tube100_asset_path = "/catkin_ws/src/ur5_robot_gripper/meshes/tube100/tube100.usd"
for i in range (7):
    for j in range(2):
        add_reference_to_stage(usd_path=tube100_asset_path, prim_path="/World/tube100/tube100_{}_{}".format(i,j))
        object_prim = stage.GetPrimAtPath("/World/tube100/tube100_{}_{}".format(i,j))
        # 添加平移：https://forums.developer.nvidia.com/t/add-and-transform-usd-assets-in-python/250701/4
        xformable = UsdGeom.Xformable(object_prim)
        xformable.SetXformOpOrder([])
        translateop = xformable.AddTranslateOp()
        translateop.Set(Gf.Vec3d(0.5 + 0.1*j,  0.075+0.025*j, 0.1+0.035*i))
        # if j:
        #     xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(180.0, 0.0, 0.0))

tray_asset_path = "/catkin_ws/src/ur5_robot_gripper/meshes/tray/tray.usd"
add_reference_to_stage(usd_path=tray_asset_path, prim_path="/World/tray")
object_prim = stage.GetPrimAtPath("/World/tray")
xformable = UsdGeom.Xformable(object_prim)
xformable.SetXformOpOrder([])
translateop = xformable.AddTranslateOp()
translateop.Set(Gf.Vec3d(0.54, 0.015, -0.01))

xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(90.0, 0.0, 0.0))

xformable.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

rack1_asset_path = "/catkin_ws/src/ur5_robot_gripper/meshes/rack/rack.usd"
add_reference_to_stage(usd_path=rack1_asset_path, prim_path="/World/rack/rack1")
object_prim = stage.GetPrimAtPath("/World/rack/rack1")
xformable = UsdGeom.Xformable(object_prim)
xformable.SetXformOpOrder([])
translateop = xformable.AddTranslateOp()
translateop.Set(Gf.Vec3d(0.4, 0.30, -0.002))

xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))

xformable.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

rack2_asset_path = "/catkin_ws/src/ur5_robot_gripper/meshes/rack/rack.usd"
add_reference_to_stage(usd_path=rack2_asset_path, prim_path="/World/rack/rack2")
object_prim = stage.GetPrimAtPath("/World/rack/rack2")
xformable = UsdGeom.Xformable(object_prim)
xformable.SetXformOpOrder([])
translateop = xformable.AddTranslateOp()
translateop.Set(Gf.Vec3d(0.4, -0.38, -0.002))

xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0.0, 0.0, 0.0))

xformable.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))

# Load demo tubes to racks

for j in range (4):
        add_reference_to_stage(usd_path=tube75_asset_path, prim_path="/World/tube75/tube75_{}_{}".format(i,j))
        object_prim = stage.GetPrimAtPath("/World/tube75/tube75_{}_{}".format(i,j))
        # 添加平移：https://forums.developer.nvidia.com/t/add-and-transform-usd-assets-in-python/250701/4
        xformable = UsdGeom.Xformable(object_prim)
        xformable.SetXformOpOrder([])
        translateop = xformable.AddTranslateOp()
        translateop.Set(Gf.Vec3d(0.4+0.013+0.016*14, -0.38+0.022+0.016*j, 0.12))
        xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(90.0, 0.0, 0.0))
for j in range (4):
        add_reference_to_stage(usd_path=tube100_asset_path, prim_path="/World/tube100/tube100_{}_{}".format(i,j))
        object_prim = stage.GetPrimAtPath("/World/tube100/tube100_{}_{}".format(i,j))
        # 添加平移：https://forums.developer.nvidia.com/t/add-and-transform-usd-assets-in-python/250701/4
        xformable = UsdGeom.Xformable(object_prim)
        xformable.SetXformOpOrder([])
        translateop = xformable.AddTranslateOp()
        translateop.Set(Gf.Vec3d(0.4+0.013 +0.016*14, 0.3+0.022+0.016*j, 0.12))
        xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(90.0, 0.0, 0.0))
# Set physics
# https://docs.omniverse.nvidia.com/isaacsim/latest/how_to_guides/environment_setup.html
# https://forums.developer.nvidia.com/t/load-meshes-into-sim/257938/2
# Create OG graph
# https://docs.omniverse.nvidia.com/isaacsim/latest/ros_tutorials/tutorial_ros_manipulation.html


# load obj:
# https://forums.developer.nvidia.com/t/how-to-conveniently-import-obj-files-via-python-scripts/204692/6

# Add Omini graph
# https://docs.omniverse.nvidia.com/isaacsim/latest/ros2_tutorials/tutorial_ros2_manipulation.html?highlight=RCLPy#ros2-joint-control-extension-python-scripting
import omni.graph.core as og
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("SubscribeJointState", "omni.isaac.ros2_bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),


            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
            ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("ArticulationController.inputs:robotPath", "/ur5_with_robotiq_2f_85"),
            ("ArticulationController.inputs:targetPrim", "/ur5_with_robotiq_2f_85"),
            ("SubscribeJointState.inputs:topicName", "/joint_states"),
        ],
    },
)

# Add camera
cameraPath = "/Camera/Camera1"
camera = UsdGeom.Camera.Define(stage, cameraPath)
camera.AddTranslateOp().Set(Gf.Vec3d(1.3, 0.35, 1.3))
camera.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(30.0, 10.0, 95.0))
demo_camera = cameraPath
# Add camera2
cameraPath = "/Camera/Camera_rack1"
camera = UsdGeom.Camera.Define(stage, cameraPath)
camera.AddTranslateOp().Set(Gf.Vec3d(0.5, -0.32, 1.4))
camera.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0., 0.0, 0.0))
demo_camera = cameraPath

# Add camera on top
cameraPath = "/Camera/Camera_on_top"
camera = UsdGeom.Camera.Define(stage, cameraPath)
camera.AddTranslateOp().Set(Gf.Vec3d(0.55, 0.015, 1.35))
camera.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(0., 0.0, 90.0))
demo_camera = cameraPath
# 抓取物体
# https://forums.developer.nvidia.com/t/object-gripping-and-picking/291963/6