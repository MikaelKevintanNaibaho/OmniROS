# gui/cmd_create_joint.py
"""
Create Joint Relationship (Manual Mode)
- Creates a VISIBLE "Joint" object in the Tree View without alignment.
- Useful for pre-defining joints before physical assembly.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from utils.hierarchy import get_parent_link
from utils.dialogs import get_choice_input, get_validated_name, show_warning
from core.joint_factory import create_joint, generate_joint_name, JOINT_TYPES


class CreateJointCommand:
    def GetResources(self):
        return {
            "Pixmap": "create_joint.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "1. Create Joint Object"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS",
                "Create a visible Joint connecting two Links (without alignment).",
            ),
        }

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()

        # Validate selection
        if len(sel) != 2:
            show_warning(
                "Selection Error",
                "Please select exactly two Links:\n"
                "1. Parent Link (Base)\n"
                "2. Child Link (The one attached)",
            )
            return

        # Resolve to link containers
        parent_link = get_parent_link(sel[0])
        child_link = get_parent_link(sel[1])

        if not parent_link or not child_link:
            show_warning(
                "Invalid Selection", "Could not find App::Part Link containers."
            )
            return

        if parent_link == child_link:
            show_warning("Error", "Parent and Child cannot be the same link.")
            return

        # Get joint type from user
        joint_type, ok = get_choice_input(
            "Joint Type", "Select Joint Type:", JOINT_TYPES, default_index=0
        )
        if not ok:
            return

        # Get joint name from user
        default_name = generate_joint_name(child_link)
        joint_name = get_validated_name(
            "Joint Name", "Joint Name:", default_text=default_name
        )
        if not joint_name:
            return

        # Create the joint object
        try:
            create_joint(parent_link, child_link, joint_name, joint_type)
            FreeCAD.ActiveDocument.recompute()
        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] Joint creation failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
