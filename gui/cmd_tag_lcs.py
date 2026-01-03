# gui/cmd_tag_lcs.py
"""
Unified command to tag LCS (or any object) with semantic role.
Opens a dialog to select role and parameters.
"""

import FreeCAD
import FreeCADGui
from PySide.QtCore import QT_TRANSLATE_NOOP
from PySide.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QComboBox,
    QLineEdit,
    QPushButton,
    QGroupBox,
    QMessageBox,
)


class TagLCSCommand:
    def GetResources(self):
        return {
            "Pixmap": "lcs_tag.svg",
            "MenuText": QT_TRANSLATE_NOOP("OmniROS", "Tag Semantic Role"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "OmniROS", "Assign semantic role to selected LCS"
            ),
        }

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()
        if not sel:
            FreeCAD.Console.PrintError("[OmniROS] Please select an object to tag\n")
            return

        # For now, enforce LCS — relax later for other objects
        if sel[0].TypeId != "PartDesign::CoordinateSystem":
            FreeCAD.Console.PrintError(
                "[OmniROS] Please select a Local Coordinate System (LCS)\n"
            )
            return

        dialog = SemanticTagDialog(sel[0])
        dialog.exec_()

    def IsActive(self):
        return FreeCAD.ActiveDocument is not None


class SemanticTagDialog(QDialog):
    ROLES = {
        "base": {"label": "Base Link Origin", "fields": []},
        "link": {"label": "Link Frame", "fields": ["link_name"]},
        "joint": {
            "label": "Joint Frame",
            "fields": ["parent_link", "child_link", "joint_type"],
        },
    }

    JOINT_TYPES = ["revolute", "prismatic", "fixed", "continuous", "floating", "planar"]

    def __init__(self, obj):
        super().__init__()
        self.obj = obj
        self.setWindowTitle("OmniROS - Assign Semantic Role")
        self.setModal(True)
        self.setup_ui()
        self.load_current_tag()

    def setup_ui(self):
        layout = QVBoxLayout()

        # Role selector
        role_layout = QHBoxLayout()
        role_layout.addWidget(QLabel("Role:"))
        self.role_combo = QComboBox()
        for key, info in self.ROLES.items():
            self.role_combo.addItem(info["label"], key)
        self.role_combo.currentIndexChanged.connect(self.on_role_changed)
        role_layout.addWidget(self.role_combo)
        layout.addLayout(role_layout)

        # Dynamic fields container
        self.fields_group = QGroupBox("Parameters")
        self.fields_layout = QVBoxLayout()
        self.fields_group.setLayout(self.fields_layout)
        layout.addWidget(self.fields_group)

        # Buttons
        button_layout = QHBoxLayout()
        self.apply_btn = QPushButton("Apply")
        self.cancel_btn = QPushButton("Cancel")
        self.apply_btn.clicked.connect(self.apply_tag)
        self.cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(self.apply_btn)
        button_layout.addWidget(self.cancel_btn)
        layout.addLayout(button_layout)

        self.setLayout(layout)
        self.on_role_changed()  # Initialize fields

    def on_role_changed(self):
        # --- FIX: Recursively clear nested layouts ---
        while self.fields_layout.count():
            item = self.fields_layout.takeAt(0)

            # Case 1: It's a direct widget (unlikely in your current setup, but safe)
            if item.widget():
                item.widget().hide()  # Hide immediately to prevent ghosting
                item.widget().deleteLater()

            # Case 2: It's a sub-layout (Your HBoxes)
            elif item.layout():
                sub_layout = item.layout()
                # Loop through the items inside the HBox (Label + Field)
                while sub_layout.count():
                    sub_item = sub_layout.takeAt(0)
                    if sub_item.widget():
                        sub_item.widget().hide()
                        sub_item.widget().deleteLater()
                # Finally, detach the sub-layout itself
                sub_layout.setParent(None)

        # --- Rebuild the UI as normal ---
        role_key = self.role_combo.currentData()
        fields = self.ROLES[role_key]["fields"]

        self.field_inputs = {}
        for field in fields:
            h_layout = QHBoxLayout()
            h_layout.addWidget(QLabel(f"{field.replace('_', ' ').title()}:"))
            if field == "joint_type":
                combo = QComboBox()
                combo.addItems(self.JOINT_TYPES)
                self.field_inputs[field] = combo
                h_layout.addWidget(combo)
            else:
                line_edit = QLineEdit()
                self.field_inputs[field] = line_edit
                h_layout.addWidget(line_edit)
            self.fields_layout.addLayout(h_layout)

        # Force the dialog to resize to fit the new content
        self.fields_group.adjustSize()
        self.adjustSize()

    def load_current_tag(self):
        """Load existing tag into dialog."""
        if hasattr(self.obj, "OmniROS_Role") and self.obj.OmniROS_Role:
            role = self.obj.OmniROS_Role
            # Find index in combo
            for i in range(self.role_combo.count()):
                if self.role_combo.itemData(i) == role:
                    self.role_combo.setCurrentIndex(i)
                    break

            # Load field values
            for field, widget in self.field_inputs.items():
                prop_name = f"OmniROS_{field.replace('_', '').capitalize()}"
                if hasattr(self.obj, prop_name):
                    value = getattr(self.obj, prop_name)
                    if isinstance(widget, QLineEdit):
                        widget.setText(str(value))
                    elif isinstance(widget, QComboBox):
                        idx = widget.findText(str(value))
                        if idx >= 0:
                            widget.setCurrentIndex(idx)

    def apply_tag(self):
        role_key = self.role_combo.currentData()
        kwargs = {}

        # Collect field values
        for field, widget in self.field_inputs.items():
            if isinstance(widget, QLineEdit):
                value = widget.text().strip()
            elif isinstance(widget, QComboBox):
                value = widget.currentText()

            if not value:
                QMessageBox.warning(
                    self, "Missing Field", f"Please enter {field.replace('_', ' ')}"
                )
                return
            kwargs[field] = value

        try:
            from core.semantic_tagger import tag_object

            tag_object(self.obj, role_key, **kwargs)
            self.accept()
        except Exception as e:
            QMessageBox.critical(self, "Tag Error", f"Failed to tag object:\n{str(e)}")
