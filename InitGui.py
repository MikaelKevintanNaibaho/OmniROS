# InitGui.py
"""
OmniROS Workbench — safe imports inside methods only.
"""

import FreeCAD
import FreeCADGui
import os


class OmniROSWorkbench(FreeCADGui.Workbench):
    MenuText = "OmniROS"
    ToolTip = "Robot Assembly and Export Workbench"

    def Initialize(self):
        import sys
        import os

        # --- 1. Robust Path Detection (The Fix) ---
        # Since __file__ and inspect failed, we check the standard install locations.

        # Check User Mod Directory (~/.local/share/FreeCAD/Mod/OmniROS)
        path_user = os.path.join(FreeCAD.getUserAppDataDir(), "Mod", "OmniROS")

        # Check System Mod Directory (/usr/share/freecad/Mod/OmniROS)
        path_system = os.path.join(FreeCAD.getHomePath(), "Mod", "OmniROS")

        if os.path.exists(path_user):
            self.wb_path = path_user
        elif os.path.exists(path_system):
            self.wb_path = path_system
        else:
            FreeCAD.Console.PrintError(
                "OmniROS Error: Could not locate workbench directory.\n"
            )
            return

        # --- 2. Setup Resources ---
        icon_dir = os.path.join(self.wb_path, "resources", "icons")
        FreeCADGui.addIconPath(icon_dir)

        # --- 3. Setup Python Import Path ---
        # Ensure we can import submodules like 'gui.place_lcs_command'
        if self.wb_path not in sys.path:
            sys.path.append(self.wb_path)

        # --- 4. Import and Register ---
        from gui.cmd_create_robot import CreateRobotCommand

        FreeCADGui.addCommand("OmniROS_CreateRobot", CreateRobotCommand())

        # Now it is safe to import
        from gui.place_lcs_command import PlaceLCSCommand

        FreeCADGui.addCommand("OmniROS_PlaceLCS", PlaceLCSCommand())

        from gui.cmd_tag_lcs import TagLCSCommand

        FreeCADGui.addCommand("OmniROS_TagLCS", TagLCSCommand())

        from gui.cmd_create_link import CreateLinkCommand

        FreeCADGui.addCommand("OmniROS_CreateLink", CreateLinkCommand())

        from gui.cmd_create_joint import CreateJointCommand

        FreeCADGui.addCommand("OmniROS_CreateJoint", CreateJointCommand())

        from gui.cmd_assemble_joint import AssembleJointCommand

        FreeCADGui.addCommand("OmniROS_AssembleJoint", AssembleJointCommand())

        tools = [
            "OmniROS_CreateRobot",
            "OmniROS_PlaceLCS",
            "OmniROS_TagLCS",
            "OmniROS_CreateLink",
            "OmniROS_CreateJoint",
            "OmniROS_AssembleJoint",
        ]
        self.appendToolbar("OmniROS Tools", tools)
        self.appendMenu("OmniROS", tools)

    def GetClassName(self):
        return "Gui::PythonWorkbench"

    def GetIcon(self):
        return "lcs_placement.svg"


# Register
if "OmniROS" not in FreeCADGui.listWorkbenches():
    FreeCADGui.addWorkbench(OmniROSWorkbench())
