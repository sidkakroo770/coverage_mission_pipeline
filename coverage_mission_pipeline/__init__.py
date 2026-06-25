"""Mission geometry preparation and orchestration package."""

from .mission_geometry_core import (
    GeometryCoreError,
    clip_partition_to_safe_area,
    create_safe_area,
    extract_polygon_components,
    prepare_partition_components,
)
from .prepared_component import (
    LocalCartesianFrame,
    PreparedComponent,
    PreparedComponentError,
    make_prepared_components,
)

__all__ = [
    "GeometryCoreError",
    "LocalCartesianFrame",
    "PreparedComponent",
    "PreparedComponentError",
    "clip_partition_to_safe_area",
    "create_safe_area",
    "extract_polygon_components",
    "make_prepared_components",
    "prepare_partition_components",
]
