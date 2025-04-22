from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'attachable_plugin_demos'

data_files = []
data_files.append(("share/ament_index/resource_index/packages", ["resource/" + package_name]))
data_files.append(("share/" + package_name, ["package.xml"]))

def package_files(directory, data_files):
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            data_files.append(("share/" + package_name + "/" + path, glob(path + "/**/*.*", recursive=True)))
    return data_files

data_files = package_files('launch/', data_files)
data_files = package_files('urdf/', data_files)
data_files = package_files('config/', data_files)
data_files = package_files('world/', data_files)
data_files = package_files('sdf/', data_files)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='author',
    maintainer_email='todo@todo.com',
    description='The ' + package_name + ' package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "attach_contact_link = attachable_plugin_demos.attach_contact_link:main",
            "arm_joint_teleop = attachable_plugin_demos.arm_joint_teleop:main",
        ],
    },
)