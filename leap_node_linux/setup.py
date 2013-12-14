from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['leap_node_linux'],
    package_dir={'': 'src'},
    requires=['geometry_msgs', 'leap_msgs', 'rospy'],
    scripts=['scripts/leap_node.py']
)

setup(**d)
