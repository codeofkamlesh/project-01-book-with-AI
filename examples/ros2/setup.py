from setuptools import setup

package_name = 'robotics_book_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=[
        'publisher_subscriber.minimal_publisher',
        'publisher_subscriber.minimal_subscriber',
        'nodes.simple_node',
        'services.add_two_ints_server',
        'services.add_two_ints_client'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/simple_launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Author',
    maintainer_email='author@example.com',
    description='Examples for the AI/Spec-Driven Robotics Book',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimal_publisher = publisher_subscriber.minimal_publisher:main',
            'minimal_subscriber = publisher_subscriber.minimal_subscriber:main',
            'simple_node = nodes.simple_node:main',
            'add_two_ints_server = services.add_two_ints_server:main',
            'add_two_ints_client = services.add_two_ints_client:main',
        ],
    },
)