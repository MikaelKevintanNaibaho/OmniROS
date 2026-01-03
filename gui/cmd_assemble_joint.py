"""
Assemble Joint Command
- Aligns a Child Link to a Parent Link by matching two LCS frames.
- Moves the 'Source' Link container.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from PySide.QtWidgets import QMessageBox


def get_link_and_local_placement(lcs_obj):
    """
    Walks up the tree from the LCS to find the first App::Part (Link Container).
    Accumulates the placement of all intermediate objects (like Bodies).

    Returns:
        (App::Part, Placement): The container and the LCS's placement relative to it.
    """
    if not lcs_obj:
        return None, None

    current_obj = lcs_obj
    accumulated_placement = lcs_obj.Placement
    found_link = None

    # Limit depth to prevent infinite loops (though strictly not possible in DAG)
    for _ in range(10):
        # Check parent(s)
        # Note: InFreeCAD, InList can have multiple parents. We assume the standard
        # hierarchy where the object belongs to one main container chain.
        parents = current_obj.InList
        if not parents:
            break

        # Prefer PartDesign::Body or App::Part
        parent = parents[0]

        # If we hit the App::Part, we are done
        if parent.TypeId == "App::Part":
            found_link = parent
            break

        # If it's an intermediate container (like a Body), accumulate placement
        if hasattr(parent, "Placement"):
            accumulated_placement = parent.Placement.multiply(accumulated_placement)

        current_obj = parent

    return found_link, accumulated_placement


class AssembleJointCommand:
    def GetResources(self):
        return {
            "Pixmap": "joint_assembly.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "2. Assemble Joint (Align LCS)"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS",
                "Select Target LCS (Parent) then Source LCS (Child) to snap them together.",
            ),
        }

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()
        if len(sel) != 2:
            QMessageBox.warning(
                None,
                "Selection Error",
                "Please select exactly two LCS frames:\n"
                "1. Target LCS (Where it goes - Parent)\n"
                "2. Source LCS (What moves - Child)",
            )
            return

        target_lcs = sel[0]  # Stays stationary
        source_lcs = sel[1]  # Moves with its container

        if (
            target_lcs.TypeId != "PartDesign::CoordinateSystem"
            or source_lcs.TypeId != "PartDesign::CoordinateSystem"
        ):
            QMessageBox.warning(
                None,
                "Type Error",
                "Both selections must be Local Coordinate Systems (LCS).",
            )
            return

        # 1. Resolve Source Link and relative offset
        child_link, child_lcs_local = get_link_and_local_placement(source_lcs)

        if not child_link:
            QMessageBox.critical(
                None,
                "Hierarchy Error",
                f"Could not find a movable App::Part container for '{source_lcs.Label}'.\n"
                "Ensure the LCS is inside a Link created by OmniROS.",
            )
            return

        # 2. Get Target Global Placement
        # We use the LCS's global placement in the document
        target_global = target_lcs.getGlobalPlacement()

        # 3. Calculate New Placement for the Child Link
        # Logic: TargetGlobal = ChildLinkNewGlobal * ChildLCS_Local
        # Valid: ChildLinkNewGlobal = TargetGlobal * ChildLCS_Local_Inverse

        new_link_placement = target_global.multiply(child_lcs_local.inverse())

        # 4. Apply with Undo Support
        try:
            FreeCAD.ActiveDocument.openTransaction("Align Joint")

            child_link.Placement = new_link_placement

            # Optional: If the child link has a "Joint Object" associated, update its label?
            # For now, just moving is sufficient.

            FreeCAD.ActiveDocument.commitTransaction()

            FreeCAD.Console.PrintMessage(
                f"[OmniROS] Snapped '{child_link.Label}' to '{target_lcs.Label}'\n"
            )
            FreeCADGui.updateGui()

        except Exception as e:
            FreeCAD.ActiveDocument.abortTransaction()
            FreeCAD.Console.PrintError(f"[OmniROS] Alignment failed: {e}\n")

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None
