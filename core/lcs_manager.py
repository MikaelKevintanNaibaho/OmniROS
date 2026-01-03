# core/lcs_manager.py
import FreeCAD


def tag_lcs_as_base(lcs_obj):
    """Tag LCS as robot base link origin."""
    _ensure_omniros_properties(lcs_obj)
    lcs_obj.OmniROS_Role = "base"
    FreeCAD.Console.PrintMessage(f"[OmniROS] Tagged '{lcs_obj.Label}' as base\n")


def tag_lcs_as_link(lcs_obj, link_name):
    """Tag LCS as link frame."""
    _ensure_omniros_properties(lcs_obj)
    lcs_obj.OmniROS_Role = "link"
    lcs_obj.OmniROS_LinkName = link_name
    FreeCAD.Console.PrintMessage(
        f"[OmniROS] Tagged '{lcs_obj.Label}' as link '{link_name}'\n"
    )


def tag_lcs_as_joint(lcs_obj, parent_link, child_link, joint_type="revolute"):
    """Tag LCS as joint frame."""
    _ensure_omniros_properties(lcs_obj)
    lcs_obj.OmniROS_Role = "joint"
    lcs_obj.OmniROS_ParentLink = parent_link
    lcs_obj.OmniROS_ChildLink = child_link
    lcs_obj.OmniROS_JointType = joint_type
    FreeCAD.Console.PrintMessage(f"[OmniROS] Tagged '{lcs_obj.Label}' as joint\n")


def _ensure_omniros_properties(obj):
    """Add OmniROS custom properties if missing."""
    if not hasattr(obj, "OmniROS_Role"):
        obj.addProperty(
            "App::PropertyString", "OmniROS_Role", "OmniROS", "Role in robot"
        )
    if not hasattr(obj, "OmniROS_LinkName"):
        obj.addProperty(
            "App::PropertyString", "OmniROS_LinkName", "OmniROS", "Link name"
        )
    if not hasattr(obj, "OmniROS_ParentLink"):
        obj.addProperty(
            "App::PropertyString", "OmniROS_ParentLink", "OmniROS", "Parent link"
        )
    if not hasattr(obj, "OmniROS_ChildLink"):
        obj.addProperty(
            "App::PropertyString", "OmniROS_ChildLink", "OmniROS", "Child link"
        )
    if not hasattr(obj, "OmniROS_JointType"):
        obj.addProperty(
            "App::PropertyEnumeration", "OmniROS_JointType", "OmniROS", "Joint type"
        )
        obj.OmniROS_JointType = ["revolute", "prismatic", "fixed", "continuous"]
