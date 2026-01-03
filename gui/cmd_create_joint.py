# gui/cmd_create_joint.py
"""
Create Joint Relationship
- Creates a VISIBLE "Joint" object in the Tree View.
- Links Parent and Child.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from PySide.QtWidgets import QInputDialog, QMessageBox


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
        # Return a standard icon or path to your .svg
        return ":/icons/preferences-system.svg"  # Standard gear icon as placeholder

    def attach(self, vobj):
        return

    def updateData(self, obj, prop):
        return


def get_link_container(obj):
    """Resolve selection to the parent App::Part link."""
    if not obj:
        return None
    if obj.TypeId == "App::Part":
        return obj
    for p in obj.InList:
        if p.TypeId == "App::Part":
            return p
    return None


class CreateJointCommand:
    def GetResources(self):
        return {
            "Pixmap": "create_joint.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "1. Create Joint Object"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS", "Create a visible Joint connecting two Links."
            ),
        }

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()
        if len(sel) != 2:
            QMessageBox.warning(
                None,
                "Selection Error",
                "Please select exactly two Links:\n1. Parent Link (Base)\n2. Child Link (The one attached)",
            )
            return

        parent_link = get_link_container(sel[0])
        child_link = get_link_container(sel[1])

        if not parent_link or not child_link:
            QMessageBox.warning(
                None, "Invalid Selection", "Could not find App::Part Link containers."
            )
            return

        if parent_link == child_link:
            QMessageBox.warning(
                None, "Error", "Parent and Child cannot be the same link."
            )
            return

        # --- Input Dialog ---
        joint_types = [
            "revolute",
            "fixed",
            "prismatic",
            "continuous",
            "floating",
            "planar",
        ]
        j_type, ok = QInputDialog.getItem(
            None, "Joint Type", "Select Joint Type:", joint_types, 0, False
        )
        if not ok:
            return

        default_name = f"Joint_{child_link.Label.replace('_link', '')}"
        j_name, ok = QInputDialog.getText(
            None, "Joint Name", "Joint Name:", text=default_name
        )
        if not ok:
            return

        # --- Create the Joint Object ---
        doc = FreeCAD.ActiveDocument

        # We use App::FeaturePython to create a custom scriptable object
        joint_obj = doc.addObject("App::FeaturePython", j_name)
        joint_obj.Label = j_name

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
        ).JointType = j_type

        # Move Joint object into the Child Link (Optional, keeps tree clean)
        # Or keep it in the root. Usually keeping it in the root or a "Joints" group is clearer.
        # Let's create a "Joints" Group if it doesn't exist.

        joints_group = doc.getObject("ROS_Joints")
        if not joints_group:
            joints_group = doc.addObject("App::DocumentObjectGroup", "ROS_Joints")
            joints_group.Label = "ROS_Joints"

        joints_group.addObject(joint_obj)

        doc.recompute()
        FreeCAD.Console.PrintMessage(f"[OmniROS] Created Joint Object: {j_name}\n")

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
