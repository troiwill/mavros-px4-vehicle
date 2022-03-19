from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mavros_px4_vehicle'],
    package_dir={'': 'src'}
)

setup(**d)
