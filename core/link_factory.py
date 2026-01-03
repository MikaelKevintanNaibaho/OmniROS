# core/link_factory.py
import FreeCAD
import FreeCADGui


def create_link_from_object(obj, link_name):
    doc = obj.Document

    # Create visual representation
    visual = _create_link_rep(obj, link_name, "visual", visible=True)
    collision = _create_link_rep(obj, link_name, "collision", visible=False)

    # Create App::Part container
    link_part = doc.addObject("App::Part", f"{link_name}_link")
    link_part.Label = f"{link_name}_link"
    link_part.addObject(visual)
    link_part.addObject(collision)

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
