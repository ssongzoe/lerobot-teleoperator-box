"""LeRobot teleoperator package for bimanual RBY1 Cartesian VR control."""

from .config_box_vr import BoxVrConfig
from .box_vr import BoxVr

__all__ = [
    "BoxVr",
    "BoxVrConfig",
]