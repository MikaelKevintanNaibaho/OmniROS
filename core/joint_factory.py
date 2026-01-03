# core/joint_factory.py
"""
Factory for creating ROS Joint objects in FreeCAD.
"""

import FreeCAD


# Supported joint types
JOINT_TYPES = [
    "revolute",
    "fixed",
    "prismatic",
    "continuous",
    "floating",
    "planar",
]


class RosJointProxy:
    """
    Proxy class for Joint FeaturePython object.
    Handles object behavior and persistence.
    """

    def __init__(self, obj):
        obj.Proxy = self

    def execute(self, obj):
        """Called when object needs recomputing."""
        pass

    def onDocumentRestored(self, obj):
        """Called when document is reopened."""
        pass


class RosJointViewProxy:
    """
    Proxy class for Joint ViewProvider.
    Handles visual representation in the tree.
    """

    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        """Return icon path for tree view."""
        return ":/icons/preferences-system.svg"

    def attach(self, vobj):
        """Called when ViewProvider is attached."""
        pass

    def updateData(self, obj, prop):
        """Called when object data changes."""
        pass


def create_joint(parent_link, child_link, joint_name, joint_type="revolute"):
    """
    Create a ROS Joint object connecting two links.

    Args:
        parent_link: App::Part representing parent link
        child_link: App::Part representing child link
        joint_name: Name for the joint
        joint_type: Type of joint (default: "revolute")

    Returns:
        App::FeaturePython: Created joint object

    Raises:
        ValueError: If joint_type is invalid
    """
    if joint_type not in JOINT_TYPES:
        raise ValueError(
            f"Invalid joint type '{joint_type}'. Valid types: {JOINT_TYPES}"
        )

    doc = parent_link.Document

    # Create FeaturePython object
    joint_obj = doc.addObject("App::FeaturePython", joint_name)
    joint_obj.Label = joint_name

    # Attach proxy objects
    RosJointProxy(joint_obj)
    if FreeCAD.GuiUp:
        RosJointViewProxy(joint_obj.ViewObject)

    # Add properties
    joint_obj.addProperty(
        "App::PropertyLink",
        "ParentLink",
        "ROS Config",
        "Parent link in the kinematic chain",
    ).ParentLink = parent_link

    joint_obj.addProperty(
        "App::PropertyLink",
        "ChildLink",
        "ROS Config",
        "Child link in the kinematic chain",
    ).ChildLink = child_link

    joint_obj.addProperty(
        "App::PropertyString",
        "JointType",
        "ROS Config",
        "Type of joint (revolute, fixed, etc.)",
    ).JointType = joint_type

    # Add to joints group
    from utils.hierarchy import find_or_create_group

    joints_group = find_or_create_group(doc, "ROS_Joints", "ROS_Joints")
    joints_group.addObject(joint_obj)

    FreeCAD.Console.PrintMessage(
        f"[OmniROS] Created Joint '{joint_name}' ({joint_type}): "
        f"{parent_link.Label} → {child_link.Label}\n"
    )

    return joint_obj


def get_all_joints(doc):
    """
    Get all ROS Joint objects in a document.

    Args:
        doc: FreeCAD document

    Returns:
        list: All joint FeaturePython objects
    """
    joints = []
    joints_group = doc.getObject("ROS_Joints")
    if joints_group:
        joints = [obj for obj in joints_group.Group if hasattr(obj, "JointType")]
    return joints


def generate_joint_name(child_link):
    """
    Generate a sensible default joint name from child link.

    Args:
        child_link: App::Part representing child link

    Returns:
        str: Suggested joint name
    """
    # Remove common suffixes
    base_name = (
        child_link.Label.replace("_link", "")
        .replace("_visual", "")
        .replace("_collision", "")
    )
    return f"joint_{base_name}"
