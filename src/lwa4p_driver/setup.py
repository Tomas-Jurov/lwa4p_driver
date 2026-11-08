import os
from glob import glob
from setuptools import setup

package_name = "lwa4p_driver"

CONSOLE_SCRIPTS = ["driver = lwa4p_driver.driver_node:main",
                   "test_move = lwa4p_driver.test_move:main",
                   "gripper = lwa4p_driver.gripper_node:main",
                   "gripper_cmd = lwa4p_driver.gripper_cmd:main",
                   "ftm = lwa4p_driver.ftm_node:main"]
# local tools that are not part of the repository (see .gitignore): only
# registered when the module exists
for _tool in ("calibrate_hand_eye", "board_center_perpendicular", "hand_eye_to_urdf"):
    if os.path.exists(os.path.join(os.path.dirname(os.path.abspath(__file__)), package_name, _tool + ".py")):
        CONSOLE_SCRIPTS.append(f"{_tool} = {package_name}.{_tool}:main")

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tomas",
    maintainer_email="tomas.jurov@stuba.sk",
    description="CANopen driver for the Schunk LWA 4P",
    license="MIT",
    entry_points={"console_scripts": CONSOLE_SCRIPTS},
)
