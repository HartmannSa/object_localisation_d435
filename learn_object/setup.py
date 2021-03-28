from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['learn_object'],
    package_dir={'': 'src'},
    scripts=[   "scripts/teach_one_pose_node.py"]
)

setup(**setup_args)
