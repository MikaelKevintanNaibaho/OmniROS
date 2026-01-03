# gui/place_lcs_command.py
"""
Place LCS Command — uses FreeCAD's native PartDesign LCS tool.
Ensures the new LCS (and its Body) ends up inside the correct Link Container.
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
    Finds the newly created LCS, renames it, and moves its Body to the target link.
    """
    doc = FreeCAD.ActiveDocument
    if not doc:
        return

    # 1. Find the newly created LCS (It will have a generic name like 'Local_CS')
    # We look for ANY LCS that doesn't follow our 'LCS_xxx' pattern yet.
    lcs_candidate = None
    existing_nums = []

    for obj in doc.Objects:
        if obj.TypeId == "PartDesign::CoordinateSystem":
            if obj.Label.startswith("LCS_"):
                try:
                    existing_nums.append(int(obj.Label[4:]))
                except ValueError:
                    pass
            else:
                # Found one that needs processing!
                lcs_candidate = obj

    if not lcs_candidate:
        return  # No new LCS found

    # 2. Generate new name
    next_num = max(existing_nums) + 1 if existing_nums else 1
    new_lcs_name = f"LCS_{next_num:03d}"

    # 3. Rename LCS
    old_label = lcs_candidate.Label
    lcs_candidate.Label = new_lcs_name

    # 4. Handle Parent Body
    # The LCS is likely inside a Body. We need to move THAT Body.
    parent_body = None
    if hasattr(lcs_candidate, "Body") and lcs_candidate.Body:
        parent_body = lcs_candidate.Body
    else:
        # Fallback: Check InList
        for p in lcs_candidate.InList:
            if p.TypeId == "PartDesign::Body":
                parent_body = p
                break

    # 5. Move to Target Link (App::Part)
    if target_link and parent_body:
        # If the body is not already in the link, add it.
        if parent_body not in target_link.Group:
            target_link.addObject(parent_body)
            FreeCAD.Console.PrintMessage(
                f"[OmniROS] Moved Body '{parent_body.Label}' into Link '{target_link.Label}'\n"
            )

    # 6. Rename Body (if it's generic)
    if parent_body and parent_body.Label.startswith("Body"):
        new_body_name = f"{new_lcs_name}_Body"
        parent_body.Label = new_body_name
        FreeCAD.Console.PrintMessage(f"[OmniROS] Renamed Body to '{new_body_name}'\n")

    FreeCAD.Console.PrintMessage(
        f"[OmniROS] Renamed '{old_label}' to '{new_lcs_name}'\n"
    )
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
            # We must know where to put the object before the user creates it.
            target_link = None
            sel = FreeCADGui.Selection.getSelection()
            if sel:
                # Find the App::Part associated with the selection
                target_link = _get_parent_link(sel[0])

            # 2. Run standard FreeCAD command (Wait for it to finish)
            FreeCADGui.runCommand("PartDesign_CoordinateSystem")

            # 3. Post-Process
            # We pass the detected 'target_link' to the cleanup function
            _post_process_lcs(target_link)

        except Exception as e:
            FreeCAD.Console.PrintError(f"[OmniROS] Error: {e}\n")

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
