"""Build and inspect a one-joint model without a viewer or device."""
import numpy as np
from pycrobot import kdl

robot = kdl.RigidBodyTree("first_model")
joint = kdl.Joint("lift", kdl.JointType.PRISMATIC)
joint.set_axis(np.array([0.0, 0.0, 1.0]))
joint.set_qpos_limits(0.0, 0.5)
robot.add_body(kdl.RigidBody("tool", joint), "world")
qpos = robot.get_home_qpos()
start, length = robot.get_qpos_addr("lift")
assert length == 1
qpos[start] = 0.25
transform = robot.get_transform(qpos, "tool")
print(transform)
np.testing.assert_allclose(transform[:3, 3], [0.0, 0.0, 0.25])
