"""
Assemble Joint Command
- Aligns a Child Link to a Parent Link by matching two LCS frames.
- Automatically creates a Joint object in the tree.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from PySide.QtWidgets import QMessageBox, QInputDialog


def get_link_and_local_placement(lcs_obj):
    """
    Walks up the tree from the LCS to find the first App::Part (Link Container).
    Accumulates the placement of all intermediate objects (like Bodies).

    Returns:
        (App::Part, Placement): The container and the LCS's placement relative to it.
    """
    if not lcs_obj:
        return None, None

    current_obj = lcs_obj
    accumulated_placement = lcs_obj.Placement
    found_link = None

    # Limit depth to prevent infinite loops
    for _ in range(10):
        parents = current_obj.InList
        if not parents:
            break

        parent = parents[0]

        # If we hit the App::Part, we are done
        if parent.TypeId == "App::Part":
            found_link = parent
            break

        # If it's an intermediate container (like a Body), accumulate placement
        if hasattr(parent, "Placement"):
            accumulated_placement = parent.Placement.multiply(accumulated_placement)

        current_obj = parent

    return found_link, accumulated_placement


class RosJointProxy:
    """Proxy class to handle the Joint Object behavior."""

    def __init__(self, obj):
        obj.Proxy = self

    def execute(self, obj):
        return

    def onDocumentRestored(self, obj):
        pass


class RosJointViewProxy:
    """Proxy class for the View Provider (Icon)."""

    def __init__(self, vobj):
        vobj.Proxy = self

    def getIcon(self):
        return ":/icons/preferences-system.svg"

    def attach(self, vobj):
        return

    def updateData(self, obj, prop):
        return


def create_joint_object(parent_link, child_link, joint_name, joint_type):
    """
    Creates a Joint object in the tree with proper properties.

    Args:
        parent_link: App::Part representing the parent link
        child_link: App::Part representing the child link
        joint_name: Name for the joint
        joint_type: Type of joint (revolute, fixed, etc.)

    Returns:
        The created joint object
    """
    doc = FreeCAD.ActiveDocument

    # Create the Joint Object
    joint_obj = doc.addObject("App::FeaturePython", joint_name)
    joint_obj.Label = joint_name

    # Attach Logic & View Proxies
    RosJointProxy(joint_obj)
    RosJointViewProxy(joint_obj.ViewObject)

    # Add Custom Properties
    joint_obj.addProperty(
        "App::PropertyLink", "ParentLink", "ROS Config"
    ).ParentLink = parent_link
    joint_obj.addProperty(
        "App::PropertyLink", "ChildLink", "ROS Config"
    ).ChildLink = child_link
    joint_obj.addProperty(
        "App::PropertyString", "JointType", "ROS Config"
    ).JointType = joint_type

    # Move Joint object into the "ROS_Joints" group
    joints_group = doc.getObject("ROS_Joints")
    if not joints_group:
        joints_group = doc.addObject("App::DocumentObjectGroup", "ROS_Joints")
        joints_group.Label = "ROS_Joints"

    joints_group.addObject(joint_obj)

    return joint_obj


class AssembleJointCommand:
    def GetResources(self):
        return {
            "Pixmap": "joint_assembly.svg",
            "MenuText": QT_TRANSLATE_NOOP(
                "OmniROS", "Assemble Joint (Align LCS + Create Joint)"
            ),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS",
                "Select Target LCS (Parent) then Source LCS (Child) to snap them together and create a Joint.",
            ),
        }

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()
        if len(sel) != 2:
            QMessageBox.warning(
                None,
                "Selection Error",
                "Please select exactly two LCS frames:\n"
                "1. Target LCS (Where it goes - Parent)\n"
                "2. Source LCS (What moves - Child)",
            )
            return

        target_lcs = sel[0]  # Stays stationary (Parent)
        source_lcs = sel[1]  # Moves with its container (Child)

        if (
            target_lcs.TypeId != "PartDesign::CoordinateSystem"
            or source_lcs.TypeId != "PartDesign::CoordinateSystem"
        ):
            QMessageBox.warning(
                None,
                "Type Error",
                "Both selections must be Local Coordinate Systems (LCS).",
            )
            return

        # 1. Resolve Parent and Child Links
        parent_link, _ = get_link_and_local_placement(target_lcs)
        child_link, child_lcs_local = get_link_and_local_placement(source_lcs)

        if not parent_link or not child_link:
            QMessageBox.critical(
                None,
                "Hierarchy Error",
                "Could not find App::Part containers for both LCS frames.\n"
                "Ensure both LCS are inside Links created by OmniROS.",
            )
            return

        if parent_link == child_link:
            QMessageBox.warning(
                None,
                "Error",
                "Both LCS frames are in the same link. Please select LCS from different links.",
            )
            return

        # 2. Get Joint Configuration from User
        joint_types = [
            "revolute",
            "fixed",
            "prismatic",
            "continuous",
            "floating",
            "planar",
        ]

        j_type, ok = QInputDialog.getItem(
            None,
            "Joint Type",
            "Select Joint Type:",
            joint_types,
            0,  # Default to "revolute"
            False,
        )
        if not ok:
            return

        # Generate smart default name
        child_name = child_link.Label.replace("_link", "")
        default_name = f"joint_{child_name}"

        j_name, ok = QInputDialog.getText(
            None, "Joint Name", "Joint Name:", text=default_name
        )
        if not ok or not j_name.strip():
            return

        joint_name = j_name.strip()

        # 3. Perform Alignment
        try:
            FreeCAD.ActiveDocument.openTransaction("Assemble Joint")

            # Calculate and apply new placement
            target_global = target_lcs.getGlobalPlacement()
            new_link_placement = target_global.multiply(child_lcs_local.inverse())
            child_link.Placement = new_link_placement

            # 4. Create Joint Object
            joint_obj = create_joint_object(parent_link, child_link, joint_name, j_type)

            FreeCAD.ActiveDocument.commitTransaction()

            FreeCAD.Console.PrintMessage(
                f"[OmniROS] ✓ Aligned '{child_link.Label}' to '{parent_link.Label}'\n"
                f"[OmniROS] ✓ Created Joint '{joint_name}' ({j_type})\n"
            )

            FreeCADGui.updateGui()

        except Exception as e:
            FreeCAD.ActiveDocument.abortTransaction()
            FreeCAD.Console.PrintError(f"[OmniROS] Assembly failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
