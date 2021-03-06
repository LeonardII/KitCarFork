# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=[
        "sensor_camera",
        "sensor_tof",
        "vehicle_simulation_link",
        "vehicle_simulation_interface",
    ],
    package_dir={"": "src"},
)

setup(**setup_args)
