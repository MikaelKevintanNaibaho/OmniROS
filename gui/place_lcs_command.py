# gui/place_lcs_command.py
"""
Place LCS Command — uses FreeCAD's native PartDesign LCS tool.
Ensures each LCS gets its own dedicated Body inside the correct Link Container.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP


def _get_parent_link(obj):
    """Recursive search for the parent App::Part (Link) of an object."""
    if not obj:
        return None

    # Check if object itself is the container
    if obj.TypeId == "App::Part":
        return obj

    # Check immediate parents (InList)
    for parent in obj.InList:
        if parent.TypeId == "App::Part":
            return parent
        # If parent is a Body, check its parents (Body -> Link)
        if parent.TypeId == "PartDesign::Body":
            for grand_parent in parent.InList:
                if grand_parent.TypeId == "App::Part":
                    return grand_parent
    return None


def _post_process_lcs(target_link):
    """
    Finds the newly created LCS, creates a dedicated Body for it,
    and moves that Body to the target link.
    """
    doc = FreeCAD.ActiveDocument
    if not doc:
        return

    # 1. Find ALL existing LCS to determine the next number
    existing_nums = []
    for obj in doc.Objects:
        if obj.TypeId == "PartDesign::CoordinateSystem":
            if obj.Label.startswith("LCS_"):
                try:
                    existing_nums.append(int(obj.Label[4:]))
                except ValueError:
                    pass

    # 2. Find the most recently created LCS (the one without our naming pattern)
    # It will likely be named "Local CS", "Local CS001", etc.
    lcs_candidate = None
    for obj in doc.Objects:
        if obj.TypeId == "PartDesign::CoordinateSystem":
            if not obj.Label.startswith("LCS_"):
                lcs_candidate = obj
                break  # Found the new one

    if not lcs_candidate:
        FreeCAD.Console.PrintWarning("[OmniROS] No new LCS found to process.\n")
        return

    # 3. Generate new name
    next_num = max(existing_nums) + 1 if existing_nums else 1
    new_lcs_name = f"LCS_{next_num:03d}"
    new_body_name = f"{new_lcs_name}_Body"

    # 4. Get the current parent Body (if any)
    old_body = None
    for parent in lcs_candidate.InList:
        if parent.TypeId == "PartDesign::Body":
            old_body = parent
            break

    # 5. Create a NEW dedicated Body for this LCS
    new_body = doc.addObject("PartDesign::Body", new_body_name)
    new_body.Label = new_body_name

    # 6. Move the LCS from old Body (if exists) to the new Body
    if old_body:
        # Remove from old body
        try:
            old_body.removeObject(lcs_candidate)
        except:
            pass

    # Add to new body
    new_body.addObject(lcs_candidate)

    # 7. Rename the LCS
    lcs_candidate.Label = new_lcs_name

    # 8. Move the new Body to the target Link (if specified)
    if target_link:
        target_link.addObject(new_body)
        FreeCAD.Console.PrintMessage(
            f"[OmniROS] Created '{new_lcs_name}' in '{new_body_name}' → '{target_link.Label}'\n"
        )
    else:
        FreeCAD.Console.PrintMessage(
            f"[OmniROS] Created '{new_lcs_name}' in '{new_body_name}' (no parent link detected)\n"
        )

    # 9. Clean up: If old_body is now empty and was auto-generated, optionally remove it
    # (Be careful not to delete Bodies that have other content)
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

            # 1. Detect Context BEFORE running command
            target_link = None
            sel = FreeCADGui.Selection.getSelection()
            if sel:
                target_link = _get_parent_link(sel[0])

            # 2. IMPORTANT: Deactivate any active Body to prevent LCS from being added to it
            active_body = FreeCADGui.ActiveDocument.ActiveView.getActiveObject("pdbody")
            if active_body:
                FreeCADGui.ActiveDocument.ActiveView.setActiveObject("pdbody", None)

            # 3. Run standard FreeCAD command
            FreeCADGui.runCommand("PartDesign_CoordinateSystem")

            # 4. Post-Process: Create dedicated Body and organize
            _post_process_lcs(target_link)

        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] Error: {e}\n")
            import traceback

            FreeCAD.Console.PrintError(traceback.format_exc())

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
