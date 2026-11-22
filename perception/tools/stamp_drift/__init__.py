"""D435i ROS header-stamp versus rosbag receive-time diagnostics."""


def analyze_path(*args, **kwargs):
    """Lazy package-level access without importing the CLI during ``-m`` use."""
    from .analyze import analyze_path as implementation
    return implementation(*args, **kwargs)


def analyze_rows(*args, **kwargs):
    """Lazy package-level access without importing the CLI during ``-m`` use."""
    from .analyze import analyze_rows as implementation
    return implementation(*args, **kwargs)


__all__ = ["analyze_path", "analyze_rows"]
