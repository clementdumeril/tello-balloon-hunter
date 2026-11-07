from glob import glob

from setuptools import setup

setup(
    name="tello_sim",
    version="0.1.0",
    packages=["tello_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/tello_sim"]),
        ("share/tello_sim", ["package.xml"]),
        ("share/tello_sim/launch", glob("launch/*.py")),
        ("share/tello_sim/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "generate_world = tello_sim.generate_world:main",
        "sim_adapter = tello_sim.sim_adapter:main",
        "test_flight = tello_sim.test_flight:main",
        "eval_localization = tello_sim.eval_localization:main",
        "record_flight = tello_sim.record_flight:main",
    ]},
)
