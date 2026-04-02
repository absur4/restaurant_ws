from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=["seu_restaurant_smach", "seu_restaurant_smach.states"],
    package_dir={"": "src"},
)

setup(**setup_args)
