from setuptools import find_packages, setup

package_name = 'ma4825_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kenzhi',
    maintainer_email='kenzhiiskandar@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = ma4825_py_pkg.reference_test:main",
            "compvis_with_ros2_script = ma4825_py_pkg.compvis_with_ros2_script:main" 
        ],
    },
)
