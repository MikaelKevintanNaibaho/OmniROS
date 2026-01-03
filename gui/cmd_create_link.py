# gui/cmd_create_link.py
import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from PySide.QtWidgets import QInputDialog, QMessageBox


class CreateLinkCommand:
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

        # FIX 1: Allow App::Part
        valid_types = ("PartDesign::Body", "Part::Feature", "App::Part")

        if obj.TypeId not in valid_types:
            QMessageBox.critical(
                FreeCADGui.getMainWindow(),
                "Invalid Selection",
                f"Selected object is '{obj.TypeId}'.\nPlease select a Body, Part, or Std Part.",
            )
            return

        # FIX 2: Relax geometry check to allow Compounds (App::Part is always Compound)
        if hasattr(obj, "Shape") and not obj.Shape.isNull():
            # We allow Solid, CompSolid, and Compound (common for imported STEP files or App::Part)
            valid_shapes = ("Solid", "CompSolid", "Compound")
            if obj.Shape.ShapeType not in valid_shapes:
                QMessageBox.warning(
                    FreeCADGui.getMainWindow(),
                    "Geometry Warning",
                    f"Selected object is a '{obj.Shape.ShapeType}'. It might not export correctly if it's not solid.",
                )
        else:
            QMessageBox.critical(
                FreeCADGui.getMainWindow(),
                "Invalid Geometry",
                "Selected object has no geometry.",
            )
            return

        name, ok = QInputDialog.getText(
            FreeCADGui.getMainWindow(),
            "OmniROS - Link Name",
            "Enter link name (e.g., 'arm_link'):",
        )
        if not (ok and name.strip()):
            return

        link_name = name.strip()
        if not link_name.replace("_", "").isalnum():
            QMessageBox.warning(
                FreeCADGui.getMainWindow(),
                "Invalid Name",
                "Link name can only contain letters, numbers, and underscores.",
            )
            return

        try:
            from core.link_factory import create_link_from_object

            results = create_link_from_object(obj, link_name)
            FreeCAD.Console.PrintMessage(f"[OmniROS] Created link '{link_name}'\n")
        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] Link creation failed: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
