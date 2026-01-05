# gui/cmd_create_link.py
"""
Create Link Command - Creates visual/collision representations from geometry.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from utils.dialogs import get_validated_name, show_error, show_warning


class CreateLinkCommand:
    # Valid source object types
    VALID_TYPES = ("PartDesign::Body", "Part::Feature", "App::Part")

    # Valid shape types for export
    VALID_SHAPES = ("Solid", "CompSolid", "Compound")

    def GetResources(self):
        return {
            "Pixmap": "create_link.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "Create Link from Selected Part"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS", "Create visual/collision/real link representations"
            ),
        }

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()

        if not sel:
            FreeCAD.Console.PrintError("[OmniROS] Please select a body or part\n")
            return

        obj = sel[0]

        # Validate object type
        if obj.TypeId not in self.VALID_TYPES:
            show_error(
                "Invalid Selection",
                f"Selected object is '{obj.TypeId}'.\n"
                f"Please select a Body, Part, or Std Part.",
            )
            return

        # Validate geometry
        if not self._validate_geometry(obj):
            return

        # Get link name from user
        link_name = get_validated_name(
            "OmniROS - Link Name", "Enter link name (e.g., 'arm_link'):"
        )
        if not link_name:
            return
        # Detect robot container from selection
        robot_container = None
        try:
            from core.robot_factory import find_parent_robot

            robot_container = find_parent_robot(obj)
        except:
            pass

        # Create the link
        try:
            from core.link_factory import create_link_from_object

            create_link_from_object(obj, link_name, robot_container)
            FreeCAD.Console.PrintMessage(f"[OmniROS] Created link '{link_name}'\n")
        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] Link creation failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def _validate_geometry(self, obj):
        """
        Validate that object has proper geometry.

        Returns:
            bool: True if valid, False otherwise
        """
        if not hasattr(obj, "Shape") or obj.Shape.isNull():
            show_error("Invalid Geometry", "Selected object has no geometry.")
            return False

        # Check shape type
        if obj.Shape.ShapeType not in self.VALID_SHAPES:
            show_warning(
                "Geometry Warning",
                f"Selected object is a '{obj.Shape.ShapeType}'. "
                "It might not export correctly if it's not solid.",
            )
            # Still return True to allow user to proceed

        return True

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
