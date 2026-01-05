# gui/cmd_export_urdf.py
"""
Export URDF Command
- Triggers the UrdfExporter for a selected robot.
- Handles directory selection and user feedback.
"""

import FreeCAD
import FreeCADGui
import os
from PySide.QtCore import QT_TRANSLATE_NOOP, Qt
from PySide.QtWidgets import QFileDialog, QApplication
from utils.dialogs import show_error, show_info, show_warning


class ExportUrdfCommand:
    def GetResources(self):
        return {
            "Pixmap": "urdf_exporter.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "Export Robot to URDF"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS", "Export the selected robot to URDF format with meshes"
            ),
        }

    def Activated(self):
        # 1. Resolve Target Robot
        robot = self._get_selected_robot()

        if not robot:
            return

        # 2. Get Output Directory
        last_dir = FreeCAD.ParamGet(
            "User parameter:BaseApp/Preferences/Mod/OmniROS"
        ).GetString("LastExportPath", os.path.expanduser("~"))

        out_dir = QFileDialog.getExistingDirectory(
            None, "Select Export Directory", last_dir
        )

        if not out_dir:
            return

        # Save path for next time
        FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/OmniROS").SetString(
            "LastExportPath", out_dir
        )

        # 3. Perform Export
        try:
            from core.exporter import UrdfExporter

            # Show wait cursor (Fixed: using QApplication instead of FreeCADGui)
            QApplication.setOverrideCursor(Qt.WaitCursor)

            exporter = UrdfExporter(robot)
            outfile = exporter.export_robot(out_dir)

            # Restore cursor
            QApplication.restoreOverrideCursor()

            show_info(
                "Export Successful",
                f"Robot '{robot.RobotName}' exported successfully!\n\n"
                f"File: {outfile}\n"
                f"Meshes: {os.path.join(out_dir, 'meshes')}",
            )

        except Exception as e:
            # Ensure cursor is restored even if error occurs
            QApplication.restoreOverrideCursor()

            show_error("Export Failed", f"An error occurred during export:\n{str(e)}")

            FreeCAD.Console.PrintError(f"[OmniROS] Export failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None

    def _get_selected_robot(self):
        """
        Determine which robot to export based on selection or document state.
        Returns: App::Part (Robot) or None
        """
        sel = FreeCADGui.Selection.getSelection()

        # Strategy 1: Check if user selected something inside a robot
        if sel:
            try:
                from core.robot_factory import find_parent_robot

                robot = find_parent_robot(sel[0])
                if robot:
                    return robot
            except:
                pass

        # Strategy 2: If no specific selection, check available robots in doc
        try:
            from core.robot_factory import get_all_robots

            robots = get_all_robots(FreeCAD.ActiveDocument)

            if len(robots) == 1:
                return robots[0]
            elif len(robots) > 1:
                show_warning(
                    "Select Robot",
                    "Multiple robots detected. Please select the specific Robot container to export.",
                )
                return None
            else:
                show_warning("No Robot", "No robot structure found in the document.")
                return None

        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] Error finding robots: {e}\n")
            return None
