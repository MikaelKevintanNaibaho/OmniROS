# gui/cmd_create_ros_package.py
import FreeCAD
import FreeCADGui
import os
from PySide.QtCore import QT_TRANSLATE_NOOP, Qt
from PySide.QtWidgets import (
    QFileDialog,
    QLineEdit,
    QDialog,
    QVBoxLayout,
    QLabel,
    QDialogButtonBox,
    QMessageBox,
    QApplication,
)
from utils.dialogs import show_warning, show_error
from core.ros2_package_generator import organize_package


class CreateRosPackageCommand:
    def GetResources(self):
        return {
            "Pixmap": "ros2_package.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "Create ROS 2 Package"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS",
                "Export robot and generate a complete ROS 2 package",
            ),
        }

    def Activated(self):
        # 1. Select Robot (Reusing logic logic)
        robot = self._get_selected_robot()
        if not robot:
            return

        # 2. Select Workspace Root (e.g. ~/ros2_ws)
        last_ws = FreeCAD.ParamGet(
            "User parameter:BaseApp/Preferences/Mod/OmniROS"
        ).GetString("LastWorkspacePath", os.path.expanduser("~"))

        workspace_root = QFileDialog.getExistingDirectory(
            None,
            "Select ROS 2 Workspace Root (contains 'src' or will be created)",
            last_ws,
        )

        if not workspace_root:
            return

        FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/OmniROS").SetString(
            "LastWorkspacePath", workspace_root
        )

        # Ensure 'src' exists
        src_dir = os.path.join(workspace_root, "src")
        if not os.path.exists(src_dir):
            # Check if user actually selected the src folder itself
            if os.path.basename(workspace_root) == "src":
                src_dir = workspace_root
                workspace_root = os.path.dirname(src_dir)
            else:
                try:
                    os.makedirs(src_dir)
                except OSError as e:
                    show_error("Error", f"Could not create 'src' directory:\n{e}")
                    return

        # 3. Get Package Name
        default_pkg_name = f"{robot.RobotName.lower()}_description"
        package_name = self._get_package_name_input(default_pkg_name)
        if not package_name:
            return

        package_path = os.path.join(src_dir, package_name)

        # 4. Run Export & Generation
        try:
            QApplication.setOverrideCursor(Qt.WaitCursor)

            # A. Export Files (URDF + Meshes) directly to package folder
            from core.exporter import UrdfExporter

            exporter = UrdfExporter(robot)

            # This creates package_path/meshes/ and package_path/robot.urdf
            urdf_full_path = exporter.export_robot(package_path)
            urdf_filename = os.path.basename(urdf_full_path)

            # B. Organize into ROS 2 Structure
            final_urdf = organize_package(package_path, package_name, urdf_filename)

            QApplication.restoreOverrideCursor()

            # Success Message
            build_cmd = f"colcon build --packages-select {package_name}"
            FreeCAD.Console.PrintMessage(
                f"[OmniROS] ✅ Package created: {package_path}\n"
            )

            msg = (
                f"ROS 2 Package '{package_name}' created successfully!\n\n"
                f"Location: {package_path}\n\n"
                f"Next Steps:\n"
                f"1. cd {workspace_root}\n"
                f"2. {build_cmd}\n"
                f"3. source install/setup.bash\n"
                f"4. ros2 launch {package_name} display.launch.py"
            )
            QMessageBox.information(None, "Package Created", msg)

        except Exception as e:
            QApplication.restoreOverrideCursor()
            show_error("Package Creation Failed", str(e))
            FreeCAD.Console.PrintError(f"[OmniROS] Error: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None

    def _get_selected_robot(self):
        # ... (Same logic as in Export URDF) ...
        sel = FreeCADGui.Selection.getSelection()
        if sel:
            try:
                from core.robot_factory import find_parent_robot

                robot = find_parent_robot(sel[0])
                if robot:
                    return robot
            except:
                pass

        from core.robot_factory import get_all_robots

        robots = get_all_robots(FreeCAD.ActiveDocument)
        if len(robots) == 1:
            return robots[0]
        elif len(robots) > 1:
            show_warning("Select Robot", "Multiple robots found. Please select one.")
            return None
        else:
            show_warning("No Robot", "No robot found in document.")
            return None

    def _get_package_name_input(self, default_name):
        dlg = QDialog()
        dlg.setWindowTitle("Package Name")
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Enter ROS 2 Package Name:"))
        line_edit = QLineEdit(default_name)
        layout.addWidget(line_edit)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        layout.addWidget(btns)
        dlg.setLayout(layout)

        if dlg.exec() == QDialog.Accepted:
            name = line_edit.text().strip()
            if not name.replace("_", "").replace("-", "").isalnum():
                show_warning("Invalid Name", "Use only alphanumerics and underscores.")
                return None
            return name
        return None
