from setuptools import find_packages, setup

package_name = 'sim2fruit_perception'

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
    maintainer='ronyfaz',
    maintainer_email='ronydahd@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pointnet_node = sim2fruit_perception.pointnet_node:main',
        'dummy_policy_node = sim2fruit_perception.dummy_policy_node:main',
        ],
    },
)
