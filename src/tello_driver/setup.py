from glob import glob

from setuptools import setup

setup(
    name="tello_driver",
    version="0.1.0",
    packages=["tello_driver"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/tello_driver"]),
        ("share/tello_driver", ["package.xml"]),
        ("share/tello_driver/launch", glob("launch/*.py")),
        ("share/tello_driver/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "tello_driver = tello_driver.driver:main",
        "tello_teleop = tello_driver.teleop:main",
        "tello_teleop_gui = tello_driver.teleop_gui:main",
        "tello_step_test = tello_driver.step_test:main",
        "tello_step_fit = tello_driver.step_fit:main",
        "tello_marker_tour = tello_driver.marker_tour:main",
        "session_recorder = tello_driver.session_recorder:main",
    ]},
)
