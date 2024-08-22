from omni.isaac.dynamic_control import _dynamic_control

dc=_dynamic_control.acquire_dynamic_control_interface()

object=dc.get_rigid_body("/World/tube75/tube75_0_0/tube75/tube75")
object_pose=dc.get_rigid_body_pose(object)

print("position:", object_pose.p)
print("rotation:", object_pose.r)

# https://forums.developer.nvidia.com/t/get-position-of-primitive/146702