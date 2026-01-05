# utils/export_utils.py
"""
Utilities for math conversions and file exports (URDF/Mesh).
"""

import FreeCAD
import Mesh
import os
import math


def format_float(val, precision=6):
    """
    Format float for URDF (clean string).
    - Removes trailing zeros.
    - Handles negative zero.
    - Precision control.
    """
    if abs(val) < 1e-9:
        return "0"
    return f"{val:.{precision}f}".rstrip("0").rstrip(".")


def get_transform_str(placement):
    """
    Convert a FreeCAD Placement to URDF origin attributes.
    URDF uses 'xyz' (Position) and 'rpy' (Fixed Axis Roll-Pitch-Yaw).

    Args:
        placement: FreeCAD.Placement object

    Returns:
        tuple: ("x y z", "r p y")
    """
    # 1. Position (XYZ)
    v = placement.Base
    xyz_str = f"{format_float(v.x / 1000.0)} {format_float(v.y / 1000.0)} {format_float(v.z / 1000.0)}"  # Note: FreeCAD uses mm, URDF uses meters. Divided by 1000.0.

    # 2. Rotation (RPY)
    # URDF uses Fixed Axis XYZ (which is equivalent to Intrinsic ZYX).
    # FreeCAD's toEuler() typically returns (yaw, pitch, roll) for ZYX.
    # We need to ensure we map them correctly.

    # 2. Rotation (RPY) - FIX: Map FreeCAD (Yaw, Pitch, Roll) to URDF (Roll, Pitch, Yaw)
    # FreeCAD .toEuler() returns (Yaw, Pitch, Roll) for the standard ZYX sequence.
    yaw, pitch, roll = placement.Rotation.toEuler()

    rpy_str = f"{format_float(yaw)} {format_float(pitch)} {format_float(roll)}"
    return xyz_str, rpy_str


def calculate_relative_placement(parent_obj, child_obj):
    """
    Calculate placement of child relative to parent.
    Useful for determining joint origins.

    Args:
        parent_obj: Parent object (App::Part or similar)
        child_obj: Child object

    Returns:
        FreeCAD.Placement: Relative placement
    """
    p_global = parent_obj.getGlobalPlacement()
    c_global = child_obj.getGlobalPlacement()

    # Relative = Parent_Inv * Child
    return p_global.inverse().multiply(c_global)


def export_mesh(obj, directory, filename):
    """
    Export object shape to STL mesh file.

    Args:
        obj: Object with a Shape attribute
        directory: Target directory path
        filename: Target filename (should end with .stl)

    Returns:
        bool: True if successful, False otherwise
    """
    full_path = os.path.join(directory, filename)

    # Ensure directory exists
    if not os.path.exists(directory):
        try:
            os.makedirs(directory)
        except OSError:
            FreeCAD.Console.PrintError(
                f"[OmniROS] Could not create directory: {directory}\n"
            )
            return False

    try:
        if not hasattr(obj, "Shape"):
            FreeCAD.Console.PrintError(
                f"[OmniROS] Object '{obj.Label}' has no Shape for mesh export.\n"
            )
            return False

        Mesh.export([obj], full_path)
        return True

    except Exception as e:
        FreeCAD.Console.PrintError(
            f"[OmniROS] STL export failed for {obj.Label}: {e}\n"
        )
        return False
