from setuptools import setup

setup(
    name="aruco_localization",
    version="0.1.0",
    packages=["aruco_localization"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/aruco_localization"]),
        ("share/aruco_localization", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "marker_map_publisher = aruco_localization.marker_map_publisher:main",
        "aruco_localizer = aruco_localization.aruco_localizer:main",
    ]},
)
