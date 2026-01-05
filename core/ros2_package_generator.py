# core/ros2_package_generator.py
"""
Generate a minimal ROS 2 package for a robot description.
"""

import os
import shutil
import re


def organize_package(package_path, package_name, urdf_name):
    """
    Organize a raw export folder into a valid ROS 2 package structure.

    Args:
        package_path: Root of the package (where urdf/meshes currently sit)
        package_name: Name of the ROS 2 package
        urdf_name: Filename of the generated URDF (e.g. 'my_robot.urdf')

    Returns:
        str: Path to the final URDF file
    """
    # 1. Create directory structure
    urdf_dir = os.path.join(package_path, "urdf")
    launch_dir = os.path.join(package_path, "launch")
    meshes_dir = os.path.join(package_path, "meshes")

    os.makedirs(urdf_dir, exist_ok=True)
    os.makedirs(launch_dir, exist_ok=True)
    os.makedirs(meshes_dir, exist_ok=True)

    # 2. Move URDF file to /urdf/
    src_urdf = os.path.join(package_path, urdf_name)
    dst_urdf = os.path.join(urdf_dir, urdf_name)

    if os.path.exists(src_urdf):
        shutil.move(src_urdf, dst_urdf)

    # 3. Update URDF paths (package://package_name/meshes)
    _update_urdf_package_path(dst_urdf, package_name)

    # 4. Write Configuration Files
    _write_package_xml(package_path, package_name)
    _write_cmakelists(package_path, package_name)
    _write_launch_file(launch_dir, package_name, urdf_name)

    return dst_urdf


def _update_urdf_package_path(urdf_path, package_name):
    """Replace file:// or old package:// with correct package://<package_name>"""
    with open(urdf_path, "r") as f:
        content = f.read()

    # Normalize: replace any existing package://.../meshes or file://.../meshes
    # with package://<package_name>/meshes
    content = re.sub(
        r'(filename=")(?:file://[^"]*?/meshes/|package://[^"]*?/meshes/)',
        f"\\1package://{package_name}/meshes/",
        content,
    )

    with open(urdf_path, "w") as f:
        f.write(content)


def _write_package_xml(package_path, package_name):
    pkg_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.1</version>
  <description>{package_name} robot description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>BSD</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    with open(os.path.join(package_path, "package.xml"), "w") as f:
        f.write(pkg_xml)


def _write_cmakelists(package_path, package_name):
    cmake = f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY urdf meshes launch
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
"""
    with open(os.path.join(package_path, "CMakeLists.txt"), "w") as f:
        f.write(cmake)


def _write_launch_file(launch_dir, package_name, urdf_filename):
    """Create a standard display.launch.py"""
    content = f"""import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = '{package_name}'
    file_subpath = 'urdf/{urdf_filename}'

    # Use xacro if needed, but for now we load raw URDF
    urdf_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'robot_description': robot_desc}}],
            arguments=[urdf_file]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # arguments=['-d', os.path.join(get_package_share_directory(pkg_name), 'config', 'view.rviz')]
        ),
    ])
"""
    with open(os.path.join(launch_dir, "display.launch.py"), "w") as f:
        f.write(content)
