# core/robot_factory.py
"""
Factory for creating Robot containers in FreeCAD.
A Robot container organizes all links and joints for a single robot instance.
"""

import FreeCAD


class RosRobotProxy:
    """
    Proxy class for Robot FeaturePython object.
    Handles robot-level behavior and persistence.
    """

    def __init__(self, obj):
        obj.Proxy = self

    def execute(self, obj):
        """Called when object needs recomputing."""
        pass

    def onDocumentRestored(self, obj):
        """Called when document is reopened."""
        pass


class RosRobotViewProxy:
    """
    Proxy class for Robot ViewProvider.
    Handles visual representation in the tree.
    """

    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        """Return icon path for tree view."""
        return "create_robot.svg"

    def attach(self, vobj):
        """Called when ViewProvider is attached."""
        pass

    def updateData(self, obj, prop):
        """Called when object data changes."""
        pass


def create_robot(robot_name, description=""):
    """
    Create a Robot container with organized structure.

    Args:
        robot_name: Name for the robot (e.g., "my_robot")
        description: Optional description

    Returns:
        dict: Dictionary containing robot container and subgroups
            {
                'container': App::Part (main robot container),
                'links_group': App::DocumentObjectGroup,
                'joints_group': App::DocumentObjectGroup
            }

    Structure created:
        Robot_<name> (App::Part)
        ├── ROS_Links (Group)
        └── ROS_Joints (Group)
    """
    doc = FreeCAD.ActiveDocument
    if not doc:
        raise RuntimeError("No active document")

    # Validate robot name
    if not robot_name or not robot_name.replace("_", "").isalnum():
        raise ValueError(
            "Robot name must contain only letters, numbers, and underscores"
        )

    # Create main robot container (App::Part)
    container_name = f"Robot_{robot_name}"
    robot_container = doc.addObject("App::Part", container_name)
    robot_container.Label = container_name

    # Add robot properties
    robot_container.addProperty(
        "App::PropertyString",
        "RobotName",
        "ROS Robot",
        "Name of the robot for URDF export",
    ).RobotName = robot_name

    robot_container.addProperty(
        "App::PropertyString", "Description", "ROS Robot", "Robot description"
    ).Description = description

    robot_container.addProperty(
        "App::PropertyString",
        "RobotType",
        "ROS Robot",
        "Type identifier (e.g., 'manipulator', 'mobile', 'humanoid')",
    ).RobotType = "manipulator"

    # Create Links subgroup
    links_group = doc.addObject("App::DocumentObjectGroup", f"{container_name}_Links")
    links_group.Label = "ROS_Links"
    robot_container.addObject(links_group)

    # Create Joints subgroup
    joints_group = doc.addObject("App::DocumentObjectGroup", f"{container_name}_Joints")
    joints_group.Label = "ROS_Joints"
    robot_container.addObject(joints_group)

    FreeCAD.Console.PrintMessage(
        f"[OmniROS] Created Robot '{robot_name}' with Links and Joints groups\n"
    )

    doc.recompute()

    return {
        "container": robot_container,
        "links_group": links_group,
        "joints_group": joints_group,
    }


def get_all_robots(doc):
    """
    Get all Robot containers in a document.

    Args:
        doc: FreeCAD document

    Returns:
        list: All robot App::Part objects
    """
    robots = []
    for obj in doc.Objects:
        if obj.TypeId == "App::Part" and hasattr(obj, "RobotName"):
            robots.append(obj)
    return robots


def get_robot_by_name(doc, robot_name):
    """
    Find a robot container by name.

    Args:
        doc: FreeCAD document
        robot_name: Name of the robot

    Returns:
        App::Part or None
    """
    container_name = f"Robot_{robot_name}"
    return doc.getObject(container_name)


def get_robot_groups(robot_container):
    """
    Get the Links and Joints groups from a robot container.

    Args:
        robot_container: Robot App::Part container

    Returns:
        dict: {'links_group': Group, 'joints_group': Group} or None
    """
    if not robot_container or robot_container.TypeId != "App::Part":
        return None

    links_group = None
    joints_group = None

    for obj in robot_container.Group:
        if obj.TypeId == "App::DocumentObjectGroup":
            if "Links" in obj.Label:
                links_group = obj
            elif "Joints" in obj.Label:
                joints_group = obj

    return {"links_group": links_group, "joints_group": joints_group}


def find_parent_robot(obj, max_depth=10):
    """
    Find the parent Robot container of an object.

    Args:
        obj: FreeCAD document object
        max_depth: Maximum depth to search

    Returns:
        App::Part (Robot container) or None
    """
    if not obj:
        return None

    # Check if object itself is a robot container
    if obj.TypeId == "App::Part" and hasattr(obj, "RobotName"):
        return obj

    # Walk up the parent chain
    current = obj
    for _ in range(max_depth):
        parents = current.InList
        if not parents:
            break

        for parent in parents:
            if parent.TypeId == "App::Part" and hasattr(parent, "RobotName"):
                return parent

        current = parents[0]

    return None
