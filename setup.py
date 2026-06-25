from setuptools import find_packages, setup

package_name = "coverage_mission_pipeline"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sidkakroo770",
    maintainer_email="sidkakroo770@users.noreply.github.com",
    description=(
        "Geometry preparation and mission orchestration for "
        "polygon coverage planning."
    ),
    license="TODO",
    tests_require=["pytest"],
)
