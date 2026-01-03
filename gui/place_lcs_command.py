# gui/place_lcs_command.py
"""
Place LCS Command — uses FreeCAD's native PartDesign LCS tool.
Ensures each LCS gets its own dedicated Body inside the correct Link Container.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from utils.hierarchy import get_parent_link, get_next_lcs_number


def _post_process_lcs(target_link):
    """
    Finds the newly created LCS, creates a dedicated Body for it,
    and moves that Body to the target link.
    """
    doc = FreeCAD.ActiveDocument
    if not doc:
        return

    # 1. Find the newly created LCS (without our naming pattern)
    lcs_candidate = None
    for obj in doc.Objects:
        if obj.TypeId == "PartDesign::CoordinateSystem":
            if not obj.Label.startswith("LCS_"):
                lcs_candidate = obj
                break

    if not lcs_candidate:
        FreeCAD.Console.PrintWarning("[OmniROS] No new LCS found to process.\n")
        return

    # 2. Generate new name
    next_num = get_next_lcs_number(doc)
    new_lcs_name = f"LCS_{next_num:03d}"
    new_body_name = f"{new_lcs_name}_Body"

    # 3. Get the current parent Body (if any)
    old_body = None
    for parent in lcs_candidate.InList:
        if parent.TypeId == "PartDesign::Body":
            old_body = parent
            break

    # 4. Create a NEW dedicated Body for this LCS
    new_body = doc.addObject("PartDesign::Body", new_body_name)
    new_body.Label = new_body_name

    # 5. Move the LCS from old Body to new Body
    if old_body:
        try:
            old_body.removeObject(lcs_candidate)
        except:
            pass

    new_body.addObject(lcs_candidate)

    # 6. Rename the LCS
    lcs_candidate.Label = new_lcs_name

    # 7. Move the new Body to the target Link
    if target_link:
        target_link.addObject(new_body)
        FreeCAD.Console.PrintMessage(
            f"[OmniROS] Created '{new_lcs_name}' in '{new_body_name}' → '{target_link.Label}'\n"
        )
    else:
        FreeCAD.Console.PrintMessage(
            f"[OmniROS] Created '{new_lcs_name}' in '{new_body_name}' (no parent link detected)\n"
        )

    # 8. Clean up empty auto-generated Body
    if old_body and len(old_body.Group) == 0 and old_body.Label.startswith("Body"):
        doc.removeObject(old_body.Name)
        FreeCAD.Console.PrintMessage(
            f"[OmniROS] Removed empty Body '{old_body.Label}'\n"
        )

    doc.recompute()
    FreeCADGui.updateGui()


class PlaceLCSCommand:
    def GetResources(self):
        return {
            "Pixmap": "lcs_placement.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "Place LCS"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS",
                "Place a Local Coordinate System on selected geometry",
            ),
        }

    def Activated(self):
        try:
            import PartDesignGui

            # 1. Detect context BEFORE running command
            target_link = None
            sel = FreeCADGui.Selection.getSelection()
            if sel:
                target_link = get_parent_link(sel[0])

            # 2. Deactivate any active Body to prevent LCS reuse
            active_body = FreeCADGui.ActiveDocument.ActiveView.getActiveObject("pdbody")
            if active_body:
                FreeCADGui.ActiveDocument.ActiveView.setActiveObject("pdbody", None)

            # 3. Run standard FreeCAD command
            FreeCADGui.runCommand("PartDesign_CoordinateSystem")

            # 4. Post-process: organize into dedicated Body
            _post_process_lcs(target_link)

        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] Error: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
