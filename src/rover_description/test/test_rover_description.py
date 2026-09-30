import os
import xml.etree.ElementTree as ET
import pytest
import xacro
from ament_index_python.packages import get_package_share_directory


@pytest.fixture(scope="module")
def urdf_tree():
    pkg_share = get_package_share_directory("rover_description")
    xacro_file = os.path.join(pkg_share, "urdf", "rover_description.urdf.xacro")
    assert os.path.exists(xacro_file), f"Xacro file not found at {xacro_file}"
    xml_doc = xacro.process_file(xacro_file)
    xml_str = xml_doc.toxml()
    return ET.fromstring(xml_str)


def test_root_link_and_hierarchy(urdf_tree):
    # Verify root link is base_link
    links = [link_elem.attrib["name"] for link_elem in urdf_tree.findall("link")]
    assert "base_link" in links, "base_link must exist"
    assert "chassis" in links, "chassis link must exist"
    assert "lidar_link" in links, "lidar_link must exist"

    # Verify base_link connects to chassis
    base_joint = urdf_tree.find('.//joint[@name="base_link_to_chassis_joint"]')
    assert base_joint is not None
    assert base_joint.find("parent").attrib["link"] == "base_link"
    assert base_joint.find("child").attrib["link"] == "chassis"


def test_base_link_rep103_transform(urdf_tree):
    # Verify base_link_to_chassis_joint has REP-103 transform
    base_joint = urdf_tree.find('.//joint[@name="base_link_to_chassis_joint"]')
    origin = base_joint.find("origin")
    assert origin is not None
    rpy = [float(x) for x in origin.attrib["rpy"].split()]
    # Roll and Yaw must rotate CAD axes so X is forward and Z is up
    assert abs(rpy[0] - (-1.570796)) < 0.01, f"Expected roll ~ -1.5708, got {rpy[0]}"
    assert abs(rpy[2] - (-1.570796)) < 0.01, f"Expected yaw ~ -1.5708, got {rpy[2]}"


def test_six_wheels_defined(urdf_tree):
    expected_wheels = [
        "front_wheel_to_rocker_left_joint",
        "middle_wheel_left_joint",
        "back_wheel_left_joint",
        "front_wheel_right_joint",
        "middle_wheel_right_joint",
        "back_wheel_right_joint",
    ]
    joints = {j.attrib["name"]: j for j in urdf_tree.findall("joint")}
    for wheel_joint in expected_wheels:
        assert wheel_joint in joints, f"Missing wheel joint {wheel_joint}"
        assert joints[wheel_joint].attrib["type"] == "continuous"


def test_suspension_joints_are_revolute_with_limits(urdf_tree):
    suspension_joints = [
        "rocker_to_chassis_left_joint",
        "bogie_to_rocker_left_joint",
        "rocker_to_chassis_right_joint",
        "rocker_to_bogie_right_joint",
    ]
    joints = {j.attrib["name"]: j for j in urdf_tree.findall("joint")}
    for s_joint in suspension_joints:
        assert s_joint in joints, f"Missing suspension joint {s_joint}"
        joint_tag = joints[s_joint]
        assert joint_tag.attrib["type"] == "revolute", f"{s_joint} must be revolute"
        limit = joint_tag.find("limit")
        assert limit is not None, f"{s_joint} must have angular limits"
        assert float(limit.attrib["lower"]) < 0.0
        assert float(limit.attrib["upper"]) > 0.0
        dynamics = joint_tag.find("dynamics")
        assert dynamics is not None, f"{s_joint} must have damping/friction dynamics"
        assert float(dynamics.attrib.get("damping", 0.0)) > 0.0


def test_chassis_com_inside_bounds(urdf_tree):
    chassis = urdf_tree.find('.//link[@name="chassis"]')
    inertial = chassis.find("inertial/origin")
    assert inertial is not None
    coords = [float(x) for x in inertial.attrib["xyz"].split()]
    # Chassis mesh bounds in CAD: X: [0.695, 1.003], Y: [-1.495, -1.319], Z: [0.768, 1.273]
    assert 0.695 <= coords[0] <= 1.003, f"Chassis COM X={coords[0]} out of bounds"
    assert -1.495 <= coords[1] <= -1.319, (
        f"Chassis COM Y={coords[1]} out of bounds (must be negative!)"
    )
    assert 0.768 <= coords[2] <= 1.273, f"Chassis COM Z={coords[2]} out of bounds"


def test_lidar_configuration_and_elevation(urdf_tree):
    # Verify lidar link has zeroed physical origin for sensor head
    lidar_link = urdf_tree.find('.//link[@name="lidar_link"]')
    vis_origins = [v.find("origin") for v in lidar_link.findall("visual")]
    assert vis_origins[0].attrib["xyz"] == "0 0 0"

    # Verify lidar joint is on differential_bar
    lidar_joint = urdf_tree.find('.//joint[@name="lidar_joint"]')
    assert lidar_joint.find("parent").attrib["link"] == "differential_bar"
    assert lidar_joint.find("child").attrib["link"] == "lidar_link"

    # Verify lidar joint elevates sensor >= 0.15m above the bar (-Y is UP in CAD)
    joint_origin = lidar_joint.find("origin")
    coords = [float(x) for x in joint_origin.attrib["xyz"].split()]
    assert coords[1] <= -0.15, (
        f"Lidar joint Y={coords[1]} should be <= -0.15m to clear all rover geometry"
    )


def test_ros2_control_tag(urdf_tree):
    control = urdf_tree.find(".//ros2_control")
    assert control is not None
    assert control.attrib["name"] == "GazeboSystem"

    # Differential bar must NOT be declared as movable joint in ros2_control
    diff_bar_ctrl = control.find('.//joint[@name="differential_bar_to_chassis_joint"]')
    assert diff_bar_ctrl is None, (
        "differential_bar_to_chassis_joint is fixed and should not be in ros2_control"
    )
