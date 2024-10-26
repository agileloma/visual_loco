import os
from setuptools import setup

package_name = 'loco_bullet'

def find_resources(package_name):
    """Find the relative path of files under the resource folder."""
    resources = []
    package_dir = os.getcwd()
    resources_dir = os.path.join(package_dir, "resource")

    for(root, _, files) in os.walk(resources_dir):
        for afile in files:
            if (afile != package_name and 
                not afile.endswith(".DS_Store") and 
                not afile.endswith(".py")):
                rel_dir = os.path.relpath(root, package_dir)
                src = os.path.join(rel_dir, afile)
                resources.append(src)

    return resources

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'resource'), resources)
    ],
    install_requires=['setuptools','pybullet'],
    zip_safe=True,
    maintainer='Li Jun',
    maintainer_email='junli@hit.edu.cn',
    description='Pybullet simulation interface for the quadrupedal robot',
    license='BSD 3-clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'SimNode = loco_bullet.SimNode:main'
        ],
    },
)
