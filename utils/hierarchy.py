# utils/hierarchy.py
"""
Hierarchy traversal utilities for navigating FreeCAD's document tree.
"""

import FreeCAD


def get_parent_link(obj, max_depth=10):
    """
    Find the parent App::Part (Link Container) of an object.

    Args:
        obj: FreeCAD document object
        max_depth: Maximum depth to search (prevents infinite loops)

    Returns:
        App::Part or None
    """
    if not obj:
        return None

    # Check if object itself is the container
    if obj.TypeId == "App::Part":
        return obj

    # Walk up the parent chain
    current = obj
    for _ in range(max_depth):
        parents = current.InList
        if not parents:
            break

        for parent in parents:
            if parent.TypeId == "App::Part":
                return parent
            # Check Body's parents
            if parent.TypeId == "PartDesign::Body":
                for grandparent in parent.InList:
                    if grandparent.TypeId == "App::Part":
                        return grandparent

        current = parents[0]

    return None


def get_link_and_local_placement(lcs_obj, max_depth=10):
    """
    Find the parent Link and calculate LCS's local placement relative to it.
    Accumulates placements through intermediate containers (Bodies).

    Args:
        lcs_obj: LCS (PartDesign::CoordinateSystem) object
        max_depth: Maximum depth to search

    Returns:
        tuple: (App::Part, Placement) or (None, None)
    """
    if not lcs_obj:
        return None, None

    current_obj = lcs_obj
    accumulated_placement = lcs_obj.Placement
    found_link = None

    for _ in range(max_depth):
        parents = current_obj.InList
        if not parents:
            break

        parent = parents[0]

        # Found the Link container
        if parent.TypeId == "App::Part":
            found_link = parent
            break

        # Accumulate placement through intermediate containers
        if hasattr(parent, "Placement"):
            accumulated_placement = parent.Placement.multiply(accumulated_placement)

        current_obj = parent

    return found_link, accumulated_placement


def get_all_lcs_in_document(doc):
    """
    Get all LCS objects in a document.

    Args:
        doc: FreeCAD document

    Returns:
        list: All PartDesign::CoordinateSystem objects
    """
    return [obj for obj in doc.Objects if obj.TypeId == "PartDesign::CoordinateSystem"]


def get_next_lcs_number(doc):
    """
    Get the next available LCS number based on existing LCS_XXX objects.

    Args:
        doc: FreeCAD document

    Returns:
        int: Next available number
    """
    existing_nums = []
    for obj in get_all_lcs_in_document(doc):
        if obj.Label.startswith("LCS_"):
            try:
                num = int(obj.Label[4:])
                existing_nums.append(num)
            except ValueError:
                pass

    return max(existing_nums) + 1 if existing_nums else 1


def find_or_create_group(doc, group_name, label=None):
    """
    Find existing group or create new one.

    Args:
        doc: FreeCAD document
        group_name: Internal name for the group
        label: Display label (defaults to group_name)

    Returns:
        App::DocumentObjectGroup
    """
    group = doc.getObject(group_name)
    if not group:
        group = doc.addObject("App::DocumentObjectGroup", group_name)
        group.Label = label or group_name
    return group
