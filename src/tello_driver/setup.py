from glob import glob

from setuptools import setup

setup(
    name="tello_driver",
    version="0.1.0",
    packages=["tello_driver", "tello_driver.core", "tello_driver.missions",
              "tello_driver.identification", "tello_driver.sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/tello_driver"]),
        ("share/tello_driver", ["package.xml"]),
        ("share/tello_driver/launch", glob("launch/*.py")),
        ("share/tello_driver/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        # core: runs on every flight (real.launch.py)
        "tello_driver = tello_driver.core.driver:main",
        "tello_ekf = tello_driver.core.ekf_node:main",
        "session_recorder = tello_driver.core.session_recorder:main",
        "tello_teleop = tello_driver.core.teleop:main",
        # missions
        "tello_marker_tour = tello_driver.missions.tour:main",
        "tello_goto = tello_driver.missions.tour:goto_main",
        "tello_patrol = tello_driver.missions.patrol:main",
        "tello_strike = tello_driver.missions.strike:main",
        # identification of the model
        "tello_step_test = tello_driver.identification.step_test:main",
        "tello_step_fit = tello_driver.identification.step_fit:main",
    ]},
)
