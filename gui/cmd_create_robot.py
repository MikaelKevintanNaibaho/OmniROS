# gui/cmd_create_robot.py
"""
Create Robot Container Command
Creates a new robot instance with organized structure.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from utils.dialogs import get_validated_name, get_text_input, show_error


class CreateRobotCommand:
    def GetResources(self):
        return {
            "Pixmap": "create_robot.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "Create Robot Container"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS", "Create a new robot instance with Links and Joints groups"
            ),
        }

    def Activated(self):
        # Get robot name from user
        robot_name = get_validated_name(
            "Create Robot",
            "Enter robot name (e.g., 'arm_robot', 'mobile_base'):",
            default_text="my_robot",
        )

        if not robot_name:
            return

        # Get optional description
        description, ok = get_text_input(
            "Robot Description", "Enter robot description (optional):", default_text=""
        )

        if not ok:
            description = ""

        # Create the robot container
        try:
            from core.robot_factory import create_robot

            result = create_robot(robot_name, description or "")

            FreeCAD.Console.PrintMessage(
                f"[OmniROS] ✓ Robot '{robot_name}' created successfully\n"
                f"[OmniROS]   - Links group: {result['links_group'].Label}\n"
                f"[OmniROS]   - Joints group: {result['joints_group'].Label}\n"
            )

            # Select the new robot container
            if FreeCAD.GuiUp:
                FreeCADGui.Selection.clearSelection()
                FreeCADGui.Selection.addSelection(result["container"])

            FreeCAD.ActiveDocument.recompute()

        except Exception as e:
            show_error("Robot Creation Failed", f"Failed to create robot:\n{str(e)}")
            FreeCAD.Console.PrintError(f"[OmniROS] Robot creation failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
