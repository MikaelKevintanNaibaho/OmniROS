# core/exporter.py
"""
URDF Exporter for OmniROS.
Traverses the robot graph, generates XML, and exports meshes.
"""

import os
import FreeCAD
import xml.dom.minidom
from xml.etree.ElementTree import Element, SubElement, tostring
from core.robot_factory import get_robot_groups
from core.joint_factory import get_all_joints
from utils.export_utils import (
    get_transform_str,
    export_mesh,
    calculate_relative_placement,
    format_float,
)


class UrdfExporter:
    def __init__(self, robot_container):
        self.robot = robot_container
        self.robot_name = robot_container.RobotName
        self.xml_root = Element("robot", name=self.robot_name)
        self.output_dir = ""
        self.mesh_dir = ""

        # Graph data
        self.links = {}  # {Label: Object}
        self.joints = []  # [Object]
        self.processed_links = set()

        self._load_structure()

    def _load_structure(self):
        """Map the robot's links and joints."""
        groups = get_robot_groups(self.robot)
        if not groups or not groups["links_group"] or not groups["joints_group"]:
            raise ValueError("Invalid robot structure: Missing Links or Joints group")

        # Index links by label
        for obj in groups["links_group"].Group:
            self.links[obj.Label] = obj

        # Get joints belonging to this robot
        self.joints = get_all_joints(self.robot.Document, self.robot)

    def export_robot(self, output_dir):
        """
        Main entry point to export the robot.

        Args:
            output_dir: Target directory path

        Returns:
            str: Path to the generated .urdf file
        """
        self.output_dir = output_dir
        self.mesh_dir = os.path.join(output_dir, "meshes")

        # Create directories
        if not os.path.exists(self.mesh_dir):
            os.makedirs(self.mesh_dir)

        FreeCAD.Console.PrintMessage(f"[OmniROS] Starting export to {output_dir}...\n")

        # 1. Identify Base Link
        base_link = self._find_base_link()
        if not base_link:
            raise RuntimeError(
                "Could not determine Base Link. Ensure graph is not cyclic."
            )

        # 2. Build Tree Recursively
        self._process_link_recursive(base_link)

        # 3. Write URDF File
        xml_str = xml.dom.minidom.parseString(tostring(self.xml_root)).toprettyxml(
            indent="  "
        )

        file_path = os.path.join(self.output_dir, f"{self.robot_name}.urdf")
        with open(file_path, "w") as f:
            f.write(xml_str)

        FreeCAD.Console.PrintMessage(f"[OmniROS] Export complete: {file_path}\n")
        return file_path

    def _find_base_link(self):
        """
        Find the root link of the robot.
        Heuristic:
        1. Look for semantic tag 'base'
        2. Look for link that is a parent but never a child
        3. Fallback: Pick first link
        """
        # 1. Semantic Tag
        for link in self.links.values():
            if hasattr(link, "OmniROS_Role") and link.OmniROS_Role == "base":
                return link

        # 2. Topology (Parent but never Child)
        child_labels = set()
        for j in self.joints:
            if j.ChildLink:
                child_labels.add(j.ChildLink.Label)

        candidates = [l for name, l in self.links.items() if name not in child_labels]

        if candidates:
            return candidates[0]

        # 3. Fallback (Cyclic graph?)
        if self.links:
            FreeCAD.Console.PrintWarning(
                "[OmniROS] Warning: Cyclic graph detected. Using arbitrary root.\n"
            )
            return list(self.links.values())[0]

        return None

    def _process_link_recursive(self, link_obj, parent_joint=None):
        """Recursive function to write link and its children."""
        if link_obj.Label in self.processed_links:
            return

        # Write Link XML
        self._write_link_xml(link_obj)
        self.processed_links.add(link_obj.Label)

        # Find joints where this link is the parent
        child_joints = [j for j in self.joints if j.ParentLink == link_obj]

        for joint in child_joints:
            child_link = joint.ChildLink
            if not child_link:
                continue

            # Write Joint XML
            self._write_joint_xml(joint)

            # Recurse
            self._process_link_recursive(child_link, joint)

    def _write_link_xml(self, link_obj):
        """Generate <link> tag and export meshes."""
        link_elem = SubElement(self.xml_root, "link", name=link_obj.Label)

        # check if this the base_link?
        # find the base link using helper method to compare
        base_link = self._find_base_link()
        is_root = (base_link is not None) and (link_obj.Label == base_link.Label)

        # Rule: Skip inertia for the root link to satisfy KDL/ROS standards
        if not is_root:
            inertial = SubElement(link_elem, "inertial")
            SubElement(inertial, "mass", value="1.0")
            SubElement(
                inertial,
                "inertia",
                ixx="0.1",
                ixy="0",
                ixz="0",
                iyy="0.1",
                iyz="0",
                izz="0.1",
            )

        # 2. Geometry (Visual/Collision)
        self._process_geometry(link_elem, link_obj, "visual")
        self._process_geometry(link_elem, link_obj, "collision")

    def _process_geometry(self, xml_parent, link_obj, geom_type):
        """
        Finds objects inside the link container matching the type (visual/collision),
        exports them as STL, and adds XML tags.
        """
        candidates = []
        for obj in link_obj.Group:
            if geom_type in obj.Label.lower() or (
                hasattr(obj, "OmniROS_Role") and obj.OmniROS_Role == geom_type
            ):
                candidates.append(obj)

        for obj in candidates:
            filename = f"{obj.Label}.stl"  # <-- Always STL

            success = export_mesh(obj, self.mesh_dir, filename)
            if not success:
                FreeCAD.Console.PrintWarning(
                    f"[OmniROS] Skipping {geom_type} geometry for {obj.Label} due to export failure.\n"
                )
                continue

            # Add XML
            geom_elem = SubElement(xml_parent, geom_type)

            # Origin relative to link
            xyz, rpy = get_transform_str(obj.Placement)
            SubElement(geom_elem, "origin", xyz=xyz, rpy=rpy)

            # Mesh tag
            geometry = SubElement(geom_elem, "geometry")
            mesh_uri = f"package://{self.robot_name}/meshes/{filename}"
            SubElement(geometry, "mesh", filename=mesh_uri, scale="0.001 0.001 0.001")

            # Material (visual only)
            if geom_type == "visual":
                mat = SubElement(geom_elem, "material", name=f"mat_{obj.Label}")
                SubElement(mat, "color", rgba="0.8 0.8 0.8 1.0")

    def _write_joint_xml(self, joint_obj):
        """Generate <joint> tag."""
        joint_elem = SubElement(
            self.xml_root, "joint", name=joint_obj.Label, type=joint_obj.JointType
        )

        SubElement(joint_elem, "parent", link=joint_obj.ParentLink.Label)
        SubElement(joint_elem, "child", link=joint_obj.ChildLink.Label)

        # Origin Calculation
        # URDF Origin is the transform from Parent Frame to Child Frame
        rel_placement = calculate_relative_placement(
            joint_obj.ParentLink, joint_obj.ChildLink
        )
        xyz, rpy = get_transform_str(rel_placement)

        SubElement(joint_elem, "origin", xyz=xyz, rpy=rpy)

        # Axis (Default to Z for revolute/prismatic/continuous)
        if joint_obj.JointType in ["revolute", "prismatic", "continuous"]:
            SubElement(joint_elem, "axis", xyz="0 0 1")

        # Limits
        if joint_obj.JointType in ["revolute", "prismatic"]:
            # TODO: Read from Joint properties if added later
            SubElement(
                joint_elem,
                "limit",
                lower="-3.14",
                upper="3.14",
                effort="100",
                velocity="1.0",
            )
