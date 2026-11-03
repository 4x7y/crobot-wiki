"""
Unit tests for pycrobot.kdl URDF loader functionality.
"""

import unittest
import numpy as np
import sys
import tempfile
import os


try:
    import pycrobot.kdl as kdl
    KDL_AVAILABLE = True
except ImportError as e:
    print(f"Warning: pycrobot.kdl not available: {e}")
    KDL_AVAILABLE = False


@unittest.skipUnless(KDL_AVAILABLE, "pycrobot.kdl module not available")
class TestUrdfLoader(unittest.TestCase):
    """Test cases for the URDF loader functionality."""

    def setUp(self):
        """Set up test fixtures."""
        # Simple valid URDF for testing
        self.simple_urdf = """<?xml version="1.0"?>
<robot name="test_robot">
  <link name="base_link">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <link name="link1">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.01"/>
    </inertial>
  </link>
  
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" velocity="1.0" effort="10.0"/>
    <dynamics damping="0.1" friction="0.01"/>
  </joint>
</robot>"""

        # Multi-joint robot URDF for more complex testing
        self.multi_joint_urdf = """<?xml version="1.0"?>
<robot name="multi_joint_robot">
  <link name="base">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.2" ixy="0" ixz="0" iyy="0.2" iyz="0" izz="0.2"/>
    </inertial>
  </link>
  
  <link name="link1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.1 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <link name="link2">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0.05 0" rpy="0 0 0"/>
      <inertia ixx="0.08" ixy="0" ixz="0" iyy="0.08" iyz="0" izz="0.08"/>
    </inertial>
  </link>
  
  <joint name="revolute_joint" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" velocity="2.0" effort="50.0"/>
    <dynamics damping="0.2" friction="0.05"/>
  </joint>
  
  <joint name="prismatic_joint" type="prismatic">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0.3 0 0" rpy="0 1.57 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.5" velocity="0.1" effort="20.0"/>
    <dynamics damping="0.1" friction="0.02"/>
  </joint>
</robot>"""

    def test_urdf_loader_import(self):
        """Test that URDF loader classes and functions are available."""
        # Check for UrdfLoader class
        self.assertTrue(hasattr(kdl, 'UrdfLoader'))
        
        # Check for convenience functions
        self.assertTrue(hasattr(kdl, 'load_urdf_file'))
        self.assertTrue(hasattr(kdl, 'load_urdf_string'))

    def test_urdf_loader_creation(self):
        """Test UrdfLoader creation."""
        loader = kdl.UrdfLoader.create()
        self.assertIsNotNone(loader)
        
        # Test constructor
        loader2 = kdl.UrdfLoader()
        self.assertIsNotNone(loader2)

    def test_load_from_string_simple(self):
        """Test loading simple URDF from string."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string(self.simple_urdf)
        
        self.assertIsNotNone(robot)
        self.assertEqual(robot.get_num_bodies(), 3)  # world + 2 URDF bodies
        self.assertEqual(robot.get_qpos_dim(), 1)  # One revolute joint
        self.assertEqual(len(loader.get_last_error()), 0)  # No error

    def test_load_from_string_with_name(self):
        """Test loading URDF from string with custom robot name."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string(self.simple_urdf, "custom_name")
        
        self.assertIsNotNone(robot)
        self.assertEqual(robot.get_num_bodies(), 3)  # world + 2 URDF bodies
        self.assertEqual(robot.get_qpos_dim(), 1)

    def test_load_from_string_multi_joint(self):
        """Test loading multi-joint URDF from string."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string(self.multi_joint_urdf)
        
        self.assertIsNotNone(robot)
        self.assertEqual(robot.get_num_bodies(), 4)  # world + 3 URDF bodies
        self.assertEqual(robot.get_qpos_dim(), 2)  # revolute + prismatic

    def test_load_invalid_xml(self):
        """Test loading invalid XML."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string("<invalid>xml content</invalid>")
        
        self.assertIsNone(robot)
        self.assertGreater(len(loader.get_last_error()), 0)
        self.assertIn("No robot element found", loader.get_last_error())

    def test_load_missing_robot_element(self):
        """Test loading XML without robot element."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string('<?xml version="1.0"?><not_robot></not_robot>')
        
        self.assertIsNone(robot)
        self.assertGreater(len(loader.get_last_error()), 0)
        self.assertIn("No robot element found", loader.get_last_error())

    def test_verify_robot_bodies(self):
        """Test that robot bodies are correctly loaded."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string(self.simple_urdf)
        
        self.assertIsNotNone(robot)
        
        # Check that bodies exist
        base_link = robot.get_body("base_link")
        link1 = robot.get_body("link1")
        
        self.assertIsNotNone(base_link)
        self.assertIsNotNone(link1)
        
        self.assertEqual(base_link.get_name(), "base_link")
        self.assertEqual(link1.get_name(), "link1")

    def test_verify_inertial_properties(self):
        """Test that inertial properties are correctly loaded."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string(self.simple_urdf)
        
        self.assertIsNotNone(robot)
        
        base_link = robot.get_body("base_link")
        link1 = robot.get_body("link1")
        
        self.assertIsNotNone(base_link)
        self.assertIsNotNone(link1)
        
        # Check masses
        self.assertAlmostEqual(base_link.get_mass(), 1.0, places=5)
        self.assertAlmostEqual(link1.get_mass(), 0.5, places=5)
        
        # Check center of mass
        com1 = link1.get_com()
        self.assertAlmostEqual(com1[0], 0.0, places=5)
        self.assertAlmostEqual(com1[1], 0.0, places=5)
        self.assertAlmostEqual(com1[2], 0.1, places=5)
        
        # Check inertia matrix
        inertia1 = link1.get_inertia_matrix()
        self.assertAlmostEqual(inertia1[0, 0], 0.05, places=5)
        self.assertAlmostEqual(inertia1[1, 1], 0.05, places=5)
        self.assertAlmostEqual(inertia1[2, 2], 0.01, places=5)
        self.assertAlmostEqual(inertia1[0, 1], 0.0, places=5)  # Off-diagonal should be zero

    def test_forward_kinematics_works(self):
        """Test that forward kinematics works with loaded robot."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string(self.simple_urdf)
        
        self.assertIsNotNone(robot)
        
        # Test forward kinematics
        qpos = robot.make_zero_qpos()
        self.assertIsNotNone(qpos)
        self.assertEqual(len(qpos), 1)
        
        # Should not raise an exception
        robot.forward_kinematics(qpos)
        
        # Test with random configuration
        qpos_random = robot.make_random_qpos()
        self.assertEqual(len(qpos_random), 1)
        robot.forward_kinematics(qpos_random)

    def test_verbose_mode(self):
        """Test verbose mode setting."""
        loader = kdl.UrdfLoader.create()
        
        # Test verbose mode doesn't crash
        loader.set_verbose(True)
        robot = loader.load_from_string(self.simple_urdf)
        self.assertIsNotNone(robot)
        
        loader.set_verbose(False)
        robot2 = loader.load_from_string(self.simple_urdf)
        self.assertIsNotNone(robot2)

    def test_package_path_setting(self):
        """Test package path setting."""
        loader = kdl.UrdfLoader.create()
        
        # Package path setting should not crash
        loader.set_package_path("test_package", "/path/to/test_package")
        loader.set_package_path("another_package", "/another/path")
        
        # Should still work after setting package paths
        robot = loader.load_from_string(self.simple_urdf)
        self.assertIsNotNone(robot)

    def test_convenience_functions(self):
        """Test convenience functions."""
        # Test load_urdf_string function
        robot1 = kdl.load_urdf_string(self.simple_urdf, "custom_name")
        self.assertIsNotNone(robot1)
        self.assertEqual(robot1.get_num_bodies(), 3)
        
        # Test with empty name (should use URDF name)
        robot2 = kdl.load_urdf_string(self.simple_urdf, "")
        self.assertIsNotNone(robot2)
        self.assertEqual(robot2.get_num_bodies(), 3)
        
        # Test verbose mode
        robot3 = kdl.load_urdf_string(self.simple_urdf, "", True)
        self.assertIsNotNone(robot3)

    def test_load_from_file(self):
        """Test loading from file."""
        loader = kdl.UrdfLoader.create()
        
        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
            f.write(self.simple_urdf)
            temp_file = f.name
        
        try:
            robot = loader.load_from_file(temp_file)
            self.assertIsNotNone(robot)
            self.assertEqual(robot.get_num_bodies(), 3)
            self.assertEqual(robot.get_qpos_dim(), 1)
        finally:
            # Clean up
            os.unlink(temp_file)

    def test_load_nonexistent_file(self):
        """Test loading nonexistent file."""
        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_file("nonexistent_file.urdf")
        
        self.assertIsNone(robot)
        self.assertGreater(len(loader.get_last_error()), 0)
        self.assertIn("Failed to open URDF file", loader.get_last_error())

    def test_convenience_file_function(self):
        """Test convenience file loading function."""
        # Create a temporary file
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
            f.write(self.simple_urdf)
            temp_file = f.name
        
        try:
            robot = kdl.load_urdf_file(temp_file)
            self.assertIsNotNone(robot)
            self.assertEqual(robot.get_num_bodies(), 3)
            
            # Test with verbose mode
            robot2 = kdl.load_urdf_file(temp_file, True)
            self.assertIsNotNone(robot2)
            self.assertEqual(robot2.get_num_bodies(), 3)
        finally:
            # Clean up
            os.unlink(temp_file)

    def test_complex_transforms(self):
        """Test loading URDF with complex transforms."""
        complex_urdf = """<?xml version="1.0"?>
<robot name="transform_robot">
  <link name="base">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <link name="rotated_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.1 0.2 0.3" rpy="0.1 0.2 0.3"/>
      <inertia ixx="0.05" ixy="0.01" ixz="0.02" iyy="0.06" iyz="0.03" izz="0.07"/>
    </inertial>
  </link>
  
  <joint name="complex_joint" type="revolute">
    <parent link="base"/>
    <child link="rotated_link"/>
    <origin xyz="1.0 2.0 3.0" rpy="0.5 1.0 1.5"/>
    <axis xyz="0.707 0.707 0"/>
    <limit lower="-2.0" upper="2.0" velocity="3.0" effort="100.0"/>
  </joint>
</robot>"""

        loader = kdl.UrdfLoader.create()
        robot = loader.load_from_string(complex_urdf)
        
        self.assertIsNotNone(robot)
        
        # Test that complex transforms are handled
        rotated_link = robot.get_body("rotated_link")
        self.assertIsNotNone(rotated_link)
        
        # Check center of mass was parsed correctly
        com = rotated_link.get_com()
        self.assertAlmostEqual(com[0], 0.1, places=5)
        self.assertAlmostEqual(com[1], 0.2, places=5)
        self.assertAlmostEqual(com[2], 0.3, places=5)
        
        # Check inertia matrix with off-diagonal terms
        inertia = rotated_link.get_inertia_matrix()
        self.assertAlmostEqual(inertia[0, 0], 0.05, places=5)
        self.assertAlmostEqual(inertia[1, 1], 0.06, places=5)
        self.assertAlmostEqual(inertia[2, 2], 0.07, places=5)
        self.assertAlmostEqual(inertia[0, 1], 0.01, places=5)
        self.assertAlmostEqual(inertia[0, 2], 0.02, places=5)
        self.assertAlmostEqual(inertia[1, 2], 0.03, places=5)
        
        # Test forward kinematics with complex transforms
        qpos = robot.make_zero_qpos()
        robot.forward_kinematics(qpos)
        
        # Test with non-zero joint angle
        qpos[0] = 1.0
        robot.forward_kinematics(qpos)

    def test_repeated_loading(self):
        """Test that repeated loading works correctly."""
        loader = kdl.UrdfLoader.create()
        
        for i in range(5):
            robot = loader.load_from_string(self.simple_urdf)
            self.assertIsNotNone(robot, f"Failed on iteration {i}")
            self.assertEqual(robot.get_num_bodies(), 3)
            self.assertEqual(robot.get_qpos_dim(), 1)

    def test_multiple_loaders(self):
        """Test that multiple loaders work independently."""
        loader1 = kdl.UrdfLoader.create()
        loader2 = kdl.UrdfLoader.create()
        
        loader1.set_verbose(True)
        loader2.set_verbose(False)
        
        robot1 = loader1.load_from_string(self.simple_urdf, "robot1")
        robot2 = loader2.load_from_string(self.multi_joint_urdf, "robot2")
        
        self.assertIsNotNone(robot1)
        self.assertIsNotNone(robot2)
        
        self.assertEqual(robot1.get_qpos_dim(), 1)
        self.assertEqual(robot2.get_qpos_dim(), 2)


if __name__ == '__main__':
    unittest.main() 