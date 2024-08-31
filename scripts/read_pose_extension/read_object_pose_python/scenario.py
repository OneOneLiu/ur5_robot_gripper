# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import distance_metrics
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_rot_matrices
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.motion_generation import ArticulationMotionPolicy, RmpFlow
from omni.isaac.motion_generation.interface_config_loader import load_supported_motion_policy_config
from omni.isaac.nucleus import get_assets_root_path

import omni
from omni.importer.urdf import _urdf
from omni.isaac.core.utils.extensions import get_extension_path_from_name
from pxr import Usd, UsdPhysics, PhysxSchema, Sdf, Gf, UsdGeom
from omni.isaac.dynamic_control import _dynamic_control

class UR_Tube_Scenario:
    def __init__(self):
        self._rmpflow = None
        self._articulation_rmpflow = None

        self._articulation = None
        self._target = None

        self._script_generator = None
        
        self.assests_root_path = "/catkin_ws/src/ur5_robot_gripper/meshes"

    def load_example_assets(self):
        """Load assets onto the stage and return them so they can be registered with the
        core.World.

        This function is called from ui_builder._setup_scene()

        The position in which things are loaded is also the position to which
        they will be returned on reset.
        """
        # Open USD assets
        omni.usd.get_context().open_stage("{}/simple_room/simple_room.usd".format(self.assests_root_path))
        self.stage = omni.usd.get_context().get_stage()
        # load URDF assets
        results, self.robot_prim_path = self.load_urdf_assets(root_path = "/catkin_ws/src/ur5_robot_gripper/urdf/ur_description", file_name = "ur5.urdf")
        # Set up the robot joint mimic
        left_knuckle_joint_path = self.robot_prim_path + "/robotiq_85_base_link/robotiq_85_left_knuckle_joint"
        left_inner_knuckle_joint_path = self.robot_prim_path + "/robotiq_85_base_link/robotiq_85_left_inner_knuckle_joint"
        left_finger_tip_joint_path = self.robot_prim_path + "/robotiq_85_left_inner_knuckle_link/robotiq_85_left_finger_tip_joint"

        self.set_mimic_joints(left_inner_knuckle_joint_path, left_knuckle_joint_path, gearing = -1.0, offset = 0.0)
        self.set_mimic_joints(left_finger_tip_joint_path, left_knuckle_joint_path, gearing = 1.0, offset = 0.0)
        
        # Load Usd assets
        ## Load the tray
        self.load_usd_assets(prim_path="/World/tray", usd_path="{}/tray/tray.usd".format(self.assests_root_path))
        self.set_object_transforms("/World/tray", (0.54, 0.015, -0.01), (90.0, 0.0, 0.0), (1, 1, 1))
        
        ## Load the tube racks
        self.load_usd_assets(prim_path="/World/rack/rack1", usd_path="{}/rack/rack.usd".format(self.assests_root_path))
        self.set_object_transforms("/World/rack/rack1", (0.4, 0.30, -0.002), (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        self.load_usd_assets(prim_path="/World/rack/rack2", usd_path="{}/rack/rack.usd".format(self.assests_root_path))
        self.set_object_transforms("/World/rack/rack2", (0.4, -0.38, -0.002), (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        
        ## Load the tubes
        tube75_asset_path = "{}/tube75/tube75.usd".format(self.assests_root_path)
        for i in range (7):
            for j in range(2):
                self.load_usd_assets(prim_path="/World/tube75/tube75_{}_{}".format(i,j), usd_path=tube75_asset_path)
                rotation = (0.0, 0.0, 0.0)
                if j:
                    rotation= (180.0, 0.0, 0.0)
                self.set_object_transforms("/World/tube75/tube75_{}_{}".format(i,j), (0.5 +0.1*j, 0.05+0.025*j, 0.05+0.035*i), rotation, (1.0, 1.0, 1.0))
        
        tube100_asset_path = "{}/tube100/tube100.usd".format(self.assests_root_path)
        for i in range (7):
            for j in range(2):
                self.load_usd_assets(prim_path="/World/tube100/tube100_{}_{}".format(i,j), usd_path=tube100_asset_path)
                self.set_object_transforms("/World/tube100/tube100_{}_{}".format(i,j), (0.5 + 0.1*j,  0.075+0.025*j, 0.1+0.035*i), (0.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        
        # Load demo tubes
        for i in range (4):
            self.load_usd_assets(prim_path="/World/tube75/tube75_demo_{}".format(i), usd_path=tube75_asset_path)
            self.set_object_transforms("/World/tube75/tube75_demo_{}".format(i), (0.4+0.013+0.016*14, -0.38+0.022+0.016*i, 0.12), (90.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        for i in range (4):
            self.load_usd_assets(prim_path="/World/tube100/tube100_demo_{}".format(i), usd_path=tube100_asset_path)
            self.set_object_transforms("/World/tube100/tube100_demo_{}".format(i), (0.4+0.013+0.016*14, 0.3+0.022+0.016*i, 0.12), (90.0, 0.0, 0.0), (1.0, 1.0, 1.0))
        
        # Add camera
        self.add_camera("/Camera/Camera1", (1.3, 0.35, 1.3), (30.0, 10.0, 95.0))
        self.add_camera("/Camera/Camera_on_top", (0.55, 0.015, 1.35), (0.0, 0.0, 90.0))
        self.add_camera("/Camera/Camera_rack1", (0.5, -0.32, 1.4), (0.0, 0.0, 0.0))
        
        # TODO: enable gpu dynamics
        self.setup_physics()
        
        return None
    
    def load_usd_assets(self, prim_path = None, usd_path = None):
        if prim_path is None or usd_path is None:
            raise ValueError("prim_path and usd_path must be provided")
        else:
            pass
        
        add_reference_to_stage(usd_path, prim_path)

    def load_urdf_assets(self, root_path = None, file_name = None):
        '''Load assets from URDF file onto the stage and return them so they can be registered with the core.World.
        '''
        self.urdf_interface = _urdf.acquire_urdf_interface()
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
        # Finally import the robot
        result, prim_path = omni.kit.commands.execute( "URDFParseAndImportFile", urdf_path="{}/{}".format(root_path, file_name),
                                                            import_config=import_config,)
        return result, prim_path
    
    def setup(self):
        """
        This function is called after assets have been loaded from ui_builder._setup_scenario().
        """
        # Set a camera view that looks good
        # set_camera_view(eye=[2, 0.8, 1], target=[0, 0, 0], camera_prim_path="/OmniverseKit_Persp")

        # Loading RMPflow can be done quickly for supported robots
        rmp_config = load_supported_motion_policy_config("Franka", "RMPflow")

        # Initialize an RmpFlow object
        # self._rmpflow = RmpFlow(**rmp_config)

        # for obstacle in self._obstacles:
        #     self._rmpflow.add_obstacle(obstacle)

        # Use the ArticulationMotionPolicy wrapper object to connect rmpflow to the Franka robot articulation.
        # self._articulation_rmpflow = ArticulationMotionPolicy(self._articulation, self._rmpflow)

        # Create a script generator to execute my_script().
        self._script_generator = self.my_script()
    
    def setup_physics(self):
        # https://docs.omniverse.nvidia.com/isaacsim/latest/how_to_guides/environment_setup.html
        # PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/World/physicsScene")) # Use this prim path will cause error, I don't understand why
        # physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(stage, "/World/physicsScene")
        PhysxSchema.PhysxSceneAPI.Apply(self.stage.GetPrimAtPath("/World"))
        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Get(self.stage, "/World")
        physxSceneAPI.CreateEnableCCDAttr(True)
        physxSceneAPI.CreateEnableStabilizationAttr(True)
        physxSceneAPI.CreateEnableGPUDynamicsAttr(True)
        physxSceneAPI.CreateBroadphaseTypeAttr("GPU")
        physxSceneAPI.CreateSolverTypeAttr("TGS")
    
    def add_camera(self, camera_prim_path, translation, rotation):
        # Add camera
        camera = UsdGeom.Camera.Define(self.stage, camera_prim_path)
        camera.AddTranslateOp().Set(Gf.Vec3d(translation))
        camera.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(rotation))
    
    def set_object_transforms(self, object_prim_path, translation, rotation, scale):
        '''Set the object transforms
        Args:
            object_prim_path (str): The path to the object prim
            translation (list): The translation values (mm)
            rotation (list): The rotation values (degree)
            scale (list): The scale values
        '''
        # TODO: Check if this is the elegant way to set the object transforms, if not, refactor the code. Becuase this method seems to transform the object into an Xformable object
        object_prim = self.stage.GetPrimAtPath(object_prim_path)
        xformable = UsdGeom.Xformable(object_prim)
        xformable.SetXformOpOrder([])
        translateop = xformable.AddTranslateOp()
        translateop.Set(Gf.Vec3d(translation))

        xformable.AddRotateXYZOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(rotation))

        xformable.AddScaleOp().Set(Gf.Vec3f(scale))
    
    def set_mimic_joints(self, mimic_joint_path, target_joint_path, gearing = 1.0, offset = 0.0):
        mimic_joint_prim = self.stage.GetPrimAtPath(mimic_joint_path)
        mimic_api = PhysxSchema.PhysxMimicJointAPI.Apply(mimic_joint_prim.GetPrim(), UsdPhysics.Tokens.rotZ)
        mimic_api.GetReferenceJointRel().AddTarget(target_joint_path)
        
        # 设置齿轮比率和偏移（根据实际需求调整）
        mimic_api.GetGearingAttr().Set(gearing)
        mimic_api.GetOffsetAttr().Set(offset)
    
    def reset(self):
        """
        This function is called when the reset button is pressed.
        In this example the core.World takes care of all necessary resetting
        by putting everything back in the position it was in when loaded.

        In more complicated scripts, e.g. scripts that modify or create USD properties
        or attributes at runtime, the user will need to implement necessary resetting
        behavior to ensure their script runs deterministically.
        """
        # Start the script over by recreating the generator.
        self._script_generator = self.my_script()

    """
    The following two functions demonstrate the mechanics of running code in a script-like way
    from a UI-based extension.  This takes advantage of Python's yield/generator framework.  

    The update() function is tied to a physics subscription, which means that it will be called
    one time on every physics step (usually 60 frames per second).  Each time it is called, it
    queries the script generator using next().  This makes the script generator execute until it hits
    a yield().  In this case, no value need be yielded.  This behavior can be nested into subroutines
    using the "yield from" keywords.
    """

    def update(self, step: float):
        try:
            result = next(self._script_generator)
        except StopIteration:
            return True

    def my_script(self):
        self.dc = _dynamic_control.acquire_dynamic_control_interface()
        obj = self.dc.get_rigid_body("/World/tube75/tube75_0_0/tube75/tube75")
        prim_path = "/World/tube75/tube75_0_0/tube75/tube75"
        target = XFormPrim(prim_path)

        # 然后你可以使用 get_world_pose()
        while True:  # 无限循环
            # pose = self.dc.get_rigid_body_pose(obj)
            translation, orientation = target.get_world_pose()
            print(f"Translation: {translation}, Orientation: {orientation}")
            # print(pose)
            yield  # 在每个仿真步骤中暂停执行，等待下一步

        # # Notice that subroutines can still use return statements to exit.  goto_position() returns a boolean to indicate success.
        # success = yield from self.goto_position(
        #     translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        # )

        # if not success:
        #     print("Could not reach target position")
        #     return

        # yield from self.open_gripper_franka(self._articulation)

        # # Visualize the new target.
        # lower_translation_target = np.array([0.4, 0, 0.04])
        # self._target.set_world_pose(lower_translation_target, orientation_target)

        # success = yield from self.goto_position(
        #     lower_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=250
        # )

        # yield from self.close_gripper_franka(self._articulation, close_position=np.array([0.02, 0.02]), atol=0.006)

        # high_translation_target = np.array([0.4, 0, 0.4])
        # self._target.set_world_pose(high_translation_target, orientation_target)

        # success = yield from self.goto_position(
        #     high_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        # )

        # next_translation_target = np.array([0.4, 0.4, 0.4])
        # self._target.set_world_pose(next_translation_target, orientation_target)

        # success = yield from self.goto_position(
        #     next_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        # )

        # next_translation_target = np.array([0.4, 0.4, 0.25])
        # self._target.set_world_pose(next_translation_target, orientation_target)

        # success = yield from self.goto_position(
        #     next_translation_target, orientation_target, self._articulation, self._rmpflow, timeout=200
        # )

        # yield from self.open_gripper_franka(self._articulation)

    ################################### Functions

    def goto_position(
        self,
        translation_target,
        orientation_target,
        articulation,
        rmpflow,
        translation_thresh=0.01,
        orientation_thresh=0.1,
        timeout=500,
    ):
        """
        Use RMPflow to move a robot Articulation to a desired task-space position.
        Exit upon timeout or when end effector comes within the provided threshholds of the target pose.
        """

        articulation_motion_policy = ArticulationMotionPolicy(articulation, rmpflow, 1 / 60)
        rmpflow.set_end_effector_target(translation_target, orientation_target)

        for i in range(timeout):
            ee_trans, ee_rot = rmpflow.get_end_effector_pose(
                articulation_motion_policy.get_active_joints_subset().get_joint_positions()
            )

            trans_dist = distance_metrics.weighted_translational_distance(ee_trans, translation_target)
            rotation_target = quats_to_rot_matrices(orientation_target)
            rot_dist = distance_metrics.rotational_distance_angle(ee_rot, rotation_target)

            done = trans_dist < translation_thresh and rot_dist < orientation_thresh

            if done:
                return True

            rmpflow.update_world()
            action = articulation_motion_policy.get_next_articulation_action(1 / 60)
            articulation.apply_action(action)

            # If not done on this frame, yield() to pause execution of this function until
            # the next frame.
            yield ()

        return False

    def open_gripper_franka(self, articulation):
        open_gripper_action = ArticulationAction(np.array([0.04, 0.04]), joint_indices=np.array([7, 8]))
        articulation.apply_action(open_gripper_action)

        # Check in once a frame until the gripper has been successfully opened.
        while not np.allclose(articulation.get_joint_positions()[7:], np.array([0.04, 0.04]), atol=0.001):
            yield ()

        return True

    def close_gripper_franka(self, articulation, close_position=np.array([0, 0]), atol=0.001):
        # To close around the cube, different values are passed in for close_position and atol
        open_gripper_action = ArticulationAction(np.array(close_position), joint_indices=np.array([7, 8]))
        articulation.apply_action(open_gripper_action)

        # Check in once a frame until the gripper has been successfully closed.
        while not np.allclose(articulation.get_joint_positions()[7:], np.array(close_position), atol=atol):
            yield ()

        return True
