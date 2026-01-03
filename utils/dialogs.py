# utils/dialogs.py
"""
Common dialog utilities for user input.
"""

from PySide.QtWidgets import QInputDialog, QMessageBox, QWidget


def get_text_input(title, prompt, default_text="", parent=None):
    """
    Show text input dialog.

    Args:
        title: Dialog title
        prompt: Prompt text
        default_text: Default value
        parent: Parent widget

    Returns:
        tuple: (text, ok) - text is None if cancelled
    """
    text, ok = QInputDialog.getText(parent, title, prompt, text=default_text)
    return (text.strip() if ok and text else None, ok)


def get_choice_input(title, prompt, choices, default_index=0, parent=None):
    """
    Show dropdown selection dialog.

    Args:
        title: Dialog title
        prompt: Prompt text
        choices: List of options
        default_index: Index of default selection
        parent: Parent widget

    Returns:
        tuple: (choice, ok) - choice is None if cancelled
    """
    choice, ok = QInputDialog.getItem(
        parent, title, prompt, choices, default_index, False
    )
    return (choice if ok else None, ok)


def show_warning(title, message, parent=None):
    """Show warning message box."""
    QMessageBox.warning(parent, title, message)


def show_error(title, message, parent=None):
    """Show error message box."""
    QMessageBox.critical(parent, title, message)


def show_info(title, message, parent=None):
    """Show information message box."""
    QMessageBox.information(parent, title, message)


def validate_name(name, allow_empty=False):
    """
    Validate a name for use in FreeCAD/ROS.

    Args:
        name: Name to validate
        allow_empty: Whether to allow empty strings

    Returns:
        tuple: (is_valid, error_message)
    """
    if not name and not allow_empty:
        return False, "Name cannot be empty"

    if not name:
        return True, None

    # Check for valid characters (alphanumeric + underscore)
    if not name.replace("_", "").isalnum():
        return False, "Name can only contain letters, numbers, and underscores"

    # ROS naming conventions
    if name[0].isdigit():
        return False, "Name cannot start with a number"

    return True, None


def get_validated_name(title, prompt, default_text="", parent=None):
    """
    Get text input with validation.

    Returns:
        str or None: Validated name, or None if cancelled
    """
    while True:
        name, ok = get_text_input(title, prompt, default_text, parent)

        if not ok:
            return None

        is_valid, error = validate_name(name)
        if is_valid:
            return name

        show_warning("Invalid Name", error, parent)
        default_text = name  # Keep the invalid input for correction
