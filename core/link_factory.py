# core/link_factory.py
import FreeCAD
import FreeCADGui


def create_link_from_object(obj, link_name, robot_container=None):
    """
    Create a link from an object.

    Args:
        obj: Source object
        link_name: Name for the link
        robot_container: Optional robot container to add link to

    Returns:
        dict: Created components
    """
    doc = obj.Document

    # Create visual representation
    visual = _create_link_rep(obj, link_name, "visual", visible=True)
    collision = _create_link_rep(obj, link_name, "collision", visible=False)

    # Create App::Part container
    link_part = doc.addObject("App::Part", f"{link_name}_link")
    link_part.Label = f"{link_name}_link"
    link_part.addObject(visual)
    link_part.addObject(collision)

    # Add to appropriate group
    _add_link_to_group(link_part, robot_container)

    doc.recompute()
    return {
        "container": link_part,
        "visual": visual,
        "collision": collision,
    }


def _create_link_rep(source, link_name, suffix, visible=True):
    doc = source.Document
    obj_type = source.TypeId

    if obj_type == "PartDesign::Body":
        # Create new Body (can hold LCS)
        new_body = doc.addObject("PartDesign::Body", f"{link_name}_{suffix}")
        new_body.Label = f"{link_name}_{suffix}"
        for feature in source.Group:
            if hasattr(feature, "Shape") and not feature.Shape.isNull():
                binder = new_body.newObject("PartDesign::ShapeBinder", feature.Name)
                binder.Support = [(feature, "")]
        _set_visibility(new_body, visible)
        return new_body

    elif obj_type in ("Part::Feature", "App::Part"):
        # Create Part::Feature (cannot hold LCS)
        if not hasattr(source, "Shape") or source.Shape.isNull():
            raise ValueError(f"{obj_type} has no valid geometry")
        new_part = doc.addObject("Part::Feature", f"{link_name}_{suffix}")
        new_part.Label = f"{link_name}_{suffix}"
        new_part.Shape = source.Shape.copy()
        _set_visibility(new_part, visible)
        return new_part

    else:
        raise ValueError(f"Unsupported object type: {obj_type}")


def _set_visibility(obj, visible):
    if hasattr(obj, "Visibility"):
        obj.Visibility = visible
    if FreeCAD.GuiUp:
        try:
            obj.ViewObject.Visibility = visible
        except Exception:
            pass


def _add_link_to_group(link_part, robot_container=None):
    """
    Add link to appropriate group (robot's Links group or global ROS_Links).

    Args:
        link_part: The link App::Part to add
        robot_container: Optional robot container
    """
    doc = link_part.Document

    if robot_container:
        # Add to robot's Links group
        from core.robot_factory import get_robot_groups

        groups = get_robot_groups(robot_container)
        if groups and groups["links_group"]:
            groups["links_group"].addObject(link_part)
            FreeCAD.Console.PrintMessage(
                f"[OmniROS] Added link to robot '{robot_container.RobotName}'\n"
            )
            return

    # Fallback: Add to global ROS_Links group (backward compatibility)
    from utils.hierarchy import find_or_create_group

    links_group = find_or_create_group(doc, "ROS_Links", "ROS_Links")
    links_group.addObject(link_part)
