# tools/ros2_package_generator.py
"""
Generate a minimal ROS 2 package for a robot description.
"""

import os
import shutil
import xml.etree.ElementTree as ET
from xml.dom import minidom


def create_ros2_package(urdf_path, meshes_dir, package_name, output_ws_dir):
    """
    Create a ROS 2 package containing the URDF and meshes.

    Args:
        urdf_path (str): Path to the exported .urdf file
        meshes_dir (str): Path to the meshes folder
        package_name (str): ROS 2 package name (must be valid)
        output_ws_dir (str): Path to the ROS 2 workspace 'src' directory

    Returns:
        str: Path to the created package
    """
    if not os.path.isdir(output_ws_dir):
        raise ValueError(f"Workspace src directory does not exist: {output_ws_dir}")
    if not os.path.isfile(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")
    if not os.path.isdir(meshes_dir):
        raise FileNotFoundError(f"Meshes directory not found: {meshes_dir}")

    package_path = os.path.join(output_ws_dir, package_name)
    os.makedirs(package_path, exist_ok=True)

    # Directories
    os.makedirs(os.path.join(package_path, "meshes"), exist_ok=True)
    os.makedirs(os.path.join(package_path, "urdf"), exist_ok=True)

    # Copy URDF to urdf/
    urdf_dest = os.path.join(package_path, "urdf", os.path.basename(urdf_path))
    shutil.copy2(urdf_path, urdf_dest)

    # Copy meshes
    for item in os.listdir(meshes_dir):
        s = os.path.join(meshes_dir, item)
        d = os.path.join(package_path, "meshes", item)
        if os.path.isfile(s):
            shutil.copy2(s, d)

    # Fix URDF to use package://<package_name>
    _update_urdf_package_path(urdf_dest, package_name)

    # Create package.xml
    _write_package_xml(package_path, package_name)

    # Create CMakeLists.txt (for ament_cmake — simpler for pure description)
    _write_cmakelists(package_path, package_name)

    return package_path


def _update_urdf_package_path(urdf_path, package_name):
    """Replace file:// or old package:// with correct package://<package_name>"""
    with open(urdf_path, "r") as f:
        content = f.read()

    # Normalize: replace any existing package://.../meshes or file://.../meshes
    # with package://<package_name>/meshes
    import re

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
  <exec_depend>rviz2</exec_depend>

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
  DIRECTORY urdf meshes
  DESTINATION share/${{PROJECT_NAME}}
)

ament_package()
"""
    with open(os.path.join(package_path, "CMakeLists.txt"), "w") as f:
        f.write(cmake)
