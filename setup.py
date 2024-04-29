from setuptools import setup

package_name = "pi2c_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["launch/pi2c_driver.launch.py"]),
    ],
    install_requires=[
        "setuptools",
        "adafruit-extended-bus",
    ],
    zip_safe=True,
    maintainer="Andy Blight",
    maintainer_email="a.j.blight@leeds.ac.uk",
    description="ROS2 driver I2C driver using Adafruit libraries",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pi2c_driver_exec = pi2c_driver.pi2c_driver:main",
        ],
    },
)
