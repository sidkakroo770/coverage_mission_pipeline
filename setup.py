from collections import defaultdict
from pathlib import Path

from setuptools import find_packages, setup

package_name = "coverage_mission_pipeline"


def recursive_data_files(source_root: str, install_root: str):
    root = Path(source_root)
    grouped = defaultdict(list)
    if not root.is_dir():
        return []
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        relative_parent = path.parent.relative_to(root)
        destination = Path(install_root) / relative_parent
        grouped[str(destination)].append(str(path))
    return sorted(grouped.items())


demo_data_files = recursive_data_files(
    "demo/noida_stage21",
    "share/coverage_mission_pipeline/demo/noida_stage21",
)

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        *demo_data_files,
        (
            "share/" + package_name + "/config",
            [
                "config/swarm_mission.example.yaml",
                "config/fleet_sitl.example.yaml",
            ],
        ),
    ],
    install_requires=["setuptools", "PyYAML", "pymavlink"],
    zip_safe=True,
    maintainer="sidkakroo770",
    maintainer_email="sidkakroo770@users.noreply.github.com",
    description=(
        "Geometry preparation, mission orchestration, and verified MAVLink "
        "fleet deployment for polygon coverage planning."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "plan_coverage_smoke_client = "
            "coverage_mission_pipeline.plan_coverage_smoke_client:main",
            "run_swarm_mission = "
            "coverage_mission_pipeline.swarm_mission_cli:main",
            "upload_swarm_missions = "
            "coverage_mission_pipeline.fleet_upload_cli:main",
            "coverage-swarm = "
            "coverage_mission_pipeline.product_cli:main",
        ],
    },
    tests_require=["pytest"],
)
