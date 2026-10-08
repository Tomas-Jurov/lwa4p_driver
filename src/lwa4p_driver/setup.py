from glob import glob
from setuptools import setup

package_name = "lwa4p_driver"

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
    entry_points={"console_scripts": ["driver = lwa4p_driver.driver_node:main",
                                      "test_move = lwa4p_driver.test_move:main",
                                      "gripper = lwa4p_driver.gripper_node:main",
                                      "gripper_cmd = lwa4p_driver.gripper_cmd:main"]},
)
