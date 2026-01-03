# core/semantic_tagger.py
"""
Generic semantic tagging for FreeCAD objects in OmniROS workbench.
Supports LCS, Parts, Bodies, or any object that can hold properties.
"""

import FreeCAD

# Known roles and their required properties
_ROLES = {
    "base": {"required": [], "description": "Robot base link origin"},
    "link": {"required": ["link_name"], "description": "Rigid link frame"},
    "joint": {
        "required": ["parent_link", "child_link", "joint_type"],
        "description": "Joint between two links",
        "enums": {
            "joint_type": [
                "revolute",
                "prismatic",
                "fixed",
                "continuous",
                "floating",
                "planar",
            ]
        },
    },
    "collision": {
        "required": ["link_name"],
        "description": "Collision geometry reference",
    },
    "visual": {"required": ["link_name"], "description": "Visual geometry reference"},
}


def _to_pascal_case(snake_str):
    """Convert snake_case to PascalCase: 'link_name' → 'LinkName'"""
    return "".join(word.capitalize() for word in snake_str.split("_"))


def tag_object(obj, role, **kwargs):
    """
    Tag a FreeCAD object with semantic meaning for OmniROS.

    Args:
        obj: FreeCAD document object
        role (str): Role (e.g., 'base', 'link', 'joint')
        **kwargs: Role-specific properties (e.g., link_name='arm')

    Raises:
        ValueError: If role is unknown or required properties missing
    """
    if role not in _ROLES:
        raise ValueError(f"Unknown role: {role}. Available: {list(_ROLES.keys())}")

    # Ensure OmniROS property group exists
    _ensure_omniros_group(obj)

    # Set role
    obj.OmniROS_Role = role

    # Validate and set role-specific properties
    role_def = _ROLES[role]
    for prop in role_def["required"]:
        if prop not in kwargs:
            raise ValueError(f"Missing required property '{prop}' for role '{role}'")
        _set_property(obj, prop, kwargs[prop], role_def.get("enums", {}))

    # Set optional properties
    for key, value in kwargs.items():
        if key not in role_def["required"]:
            _set_property(obj, key, value)

    FreeCAD.Console.PrintMessage(
        f"[OmniROS] Tagged '{obj.Label}' as {role} ({', '.join(f'{k}={v}' for k, v in kwargs.items())})\n"
    )


def _ensure_omniros_group(obj):
    """Add OmniROS property group if missing."""
    if not hasattr(obj, "OmniROS_Role"):
        obj.addProperty(
            "App::PropertyString", "OmniROS_Role", "OmniROS", "Semantic role in robot"
        )
    # Add common properties proactively (optional but helpful)
    _add_if_missing(obj, "App::PropertyString", "link_name")
    _add_if_missing(obj, "App::PropertyString", "parent_link")
    _add_if_missing(obj, "App::PropertyString", "child_link")
    _add_if_missing(obj, "App::PropertyEnumeration", "joint_type", ["revolute"])


def _add_if_missing(obj, prop_type, name, options=None):
    """Add property only if it doesn't exist."""
    internal_name = f"OmniROS_{_to_pascal_case(name)}"
    if not hasattr(obj, internal_name):
        obj.addProperty(
            prop_type, internal_name, "OmniROS", f"{name.replace('_', ' ').title()}"
        )
        if options:
            setattr(obj, internal_name, options)


def _set_property(obj, key, value, enums=None):
    """Set property with enum handling."""
    prop_name = f"OmniROS_{_to_pascal_case(key)}"
    if enums and key in enums:
        # Validate enum
        if value not in enums[key]:
            raise ValueError(
                f"Invalid value '{value}' for {key}. Options: {enums[key]}"
            )
        setattr(obj, prop_name, value)
    else:
        setattr(obj, prop_name, str(value))
