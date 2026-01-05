"""
Assemble Joint Command
- Aligns a Child Link to a Parent Link by matching two LCS frames.
- Automatically creates a Joint object in the tree.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from utils.hierarchy import get_link_and_local_placement
from utils.dialogs import get_choice_input, get_validated_name, show_warning, show_error
from core.joint_factory import create_joint, generate_joint_name, JOINT_TYPES


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

        # Validate selection
        if len(sel) != 2:
            show_warning(
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
            show_warning(
                "Type Error", "Both selections must be Local Coordinate Systems (LCS)."
            )
            return

        # Resolve parent and child links
        parent_link, _ = get_link_and_local_placement(target_lcs)
        child_link, child_lcs_local = get_link_and_local_placement(source_lcs)

        if not parent_link or not child_link:
            show_error(
                "Hierarchy Error",
                "Could not find App::Part containers for both LCS frames.\n"
                "Ensure both LCS are inside Links created by OmniROS.",
            )
            return

        if parent_link == child_link:
            show_warning(
                "Error",
                "Both LCS frames are in the same link. Please select LCS from different links.",
            )
            return

        # Get joint type from user
        joint_type, ok = get_choice_input(
            "Joint Type",
            "Select Joint Type:",
            JOINT_TYPES,
            default_index=0,  # "revolute"
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

        # Perform alignment and create joint
        try:
            FreeCAD.ActiveDocument.openTransaction("Assemble Joint")

            # Calculate and apply new placement
            target_global = target_lcs.getGlobalPlacement()
            new_link_placement = target_global.multiply(child_lcs_local.inverse())
            child_link.Placement = new_link_placement

            # Detect robot container
            robot_container = None
            try:
                from core.robot_factory import find_parent_robot

                robot_container = find_parent_robot(parent_link)
            except:
                pass

            # Create joint object
            create_joint(
                parent_link, child_link, joint_name, joint_type, robot_container
            )

            FreeCAD.ActiveDocument.commitTransaction()

            FreeCAD.Console.PrintMessage(
                f"[OmniROS] ✓ Aligned '{child_link.Label}' to '{parent_link.Label}'\n"
            )

            FreeCADGui.updateGui()

        except Exception as e:
            FreeCAD.ActiveDocument.abortTransaction()
            FreeCAD.Console.PrintError(f"[OmniROS] Assembly failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
