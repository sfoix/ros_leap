from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    #packages=['simple_robot_control'],
    #package_dir={'': 'src'},
    requires=['geometry_msgs', 'leap_msgs', 'rospy', 'simple_robot_control', 'visualization_msgs'],
    scripts=['scripts/pose_control_node.py',
             'scripts/gripper_control_node.py']
)

setup(**d)
