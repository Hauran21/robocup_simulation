from setuptools import find_packages, setup
import glob, os

package_name = 'motorctrl'

def package_files(directory, regex='**'):
    paths = []
    for path in glob.glob(os.path.join(directory, regex), recursive=True):
        if os.path.isfile(path):
            paths.append(path)
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), package_files('launch/**', '*.launch.py')),
        (os.path.join('share', package_name, 'config'), package_files('config/**', '*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robu',
    maintainer_email='li@htl-kaindorf.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motorconfig=motorctrl.motorconfig_node:main_motorconfig",
            "diffdrive=motorctrl.diffdrive_node:main_diffdrive",
            "diffdrive_drop=motorctrl.diffdrive_drop_node:main_diffdrive_drop",
            "drop=motorctrl.drop_node:main_drop",
            "diffdrive_config=motorctrl.diffdrive_config_node:main_diffdrive_config",
            "dummy=motorctrl.dummy_node:main_dummy",
        ],
    },
)
