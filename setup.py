from setuptools import find_packages, setup
import glob

package_name = 'special_implementation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob.glob('launch/*')),
        ('share/' + package_name, glob.glob('urdf/*')),
        ('share/' + package_name, glob.glob('robot/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jasdeep',
    maintainer_email='jasdeepbajaj3@tamu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['simulator = special_implementation.simulator:main',
                            'velocity_translator = special_implementation.velocity_translator:main',
                            'prm_controller = special_implementation.prm_controller:main'

        ],
    },
)
