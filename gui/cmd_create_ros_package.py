# gui/cmd_create_ros_package.py
import FreeCAD
import FreeCADGui
import os
from PySide.QtCore import QT_TRANSLATE_NOOP
from PySide.QtWidgets import (
    QFileDialog,
    QLineEdit,
    QDialog,
    QVBoxLayout,
    QLabel,
    QDialogButtonBox,
    QMessageBox,
)
from core.ros2_package_generator import create_ros2_package


class CreateRosPackageCommand:
    def GetResources(self):
        return {
            "Pixmap": "ros2_package.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "Create ROS 2 Package"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS",
                "Generate a ROS 2 package for this robot in a custom workspace",
            ),
        }

    def Activated(self):
        # Get last exported URDF path
        param = FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/OmniROS")
        urdf_path = param.GetString("LastExportUrdfPath", "")
        if not urdf_path or not os.path.isfile(urdf_path):
            FreeCAD.Console.PrintError(
                "[OmniROS] No recent URDF export found. Please export first.\n"
            )
            return

        meshes_dir = os.path.join(os.path.dirname(urdf_path), "meshes")
        robot_name = os.path.splitext(os.path.basename(urdf_path))[0]

        # Ask user to select **workspace root directory** (e.g., ~/my_robot_ws)
        default_ws = os.path.expanduser(f"~/{robot_name}_ws")
        workspace_root = QFileDialog.getExistingDirectory(
            None,
            "Select ROS 2 Workspace Root Directory",
            default_ws,  # suggest a sensible default
        )

        if not workspace_root:
            return  # user canceled

        # Auto-create or validate 'src' inside workspace_root
        src_dir = os.path.join(workspace_root, "src")
        if not os.path.exists(src_dir):
            try:
                os.makedirs(src_dir)
                FreeCAD.Console.PrintMessage(
                    f"[OmniROS] Created 'src' directory at: {src_dir}\n"
                )
            except OSError as e:
                FreeCAD.Console.PrintError(
                    f"[OmniROS] Failed to create 'src' directory: {e}\n"
                )
                return

        # Prompt for package name (default: robot_name + '_description')
        package_name = f"{robot_name}_description"
        dlg = QDialog()
        dlg.setWindowTitle("ROS 2 Package Name")
        layout = QVBoxLayout()
        layout.addWidget(QLabel("Package name:"))
        line_edit = QLineEdit(package_name)
        layout.addWidget(line_edit)
        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        btns.accepted.connect(dlg.accept)
        btns.rejected.connect(dlg.reject)
        layout.addWidget(btns)
        dlg.setLayout(layout)

        if dlg.exec() != QDialog.Accepted:
            return

        package_name = line_edit.text().strip()
        if not package_name:
            QMessageBox.warning(None, "Invalid Name", "Package name cannot be empty.")
            return

        # Validate package name (basic)
        if not package_name.replace("_", "").replace("-", "").isalnum():
            QMessageBox.warning(
                None,
                "Invalid Name",
                "Package name must contain only letters, numbers, underscores, or hyphens.",
            )
            return

        try:
            # Create package in workspace_root/src/
            pkg_path = create_ros2_package(urdf_path, meshes_dir, package_name, src_dir)

            FreeCAD.Console.PrintMessage(
                f"[OmniROS] ✅ ROS 2 package created at:\n  {pkg_path}\n"
            )

            # Print next steps
            rel_path = os.path.relpath(workspace_root, os.path.expanduser("~"))
            build_cmd = (
                f"cd ~/{rel_path} && colcon build --packages-select {package_name}"
            )
            FreeCAD.Console.PrintMessage(
                f"[OmniROS] 💡 Next steps:\n"
                f"  1. Build: {build_cmd}\n"
                f"  2. Source: source install/setup.bash\n"
                f"  3. Launch: ros2 launch {package_name} display.launch.py\n"
            )

        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] ❌ Package creation failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
