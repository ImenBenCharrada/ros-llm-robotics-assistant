from setuptools import find_packages, setup

package_name = 'llm_robotics_assistant'

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
    maintainer='imen',
    maintainer_email='imen@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'llm_planner_node = llm_robotics_assistant.nodes.llm_planner_node:main',
            'nav_node_mock = llm_robotics_assistant.nodes.mocks.nav_node_mock:main',
            'vision_node_mock = llm_robotics_assistant.nodes.mocks.vision_node_mock:main',
            'manip_node_mock = llm_robotics_assistant.nodes.mocks.manip_node_mock:main',
            'dry_run_executor = llm_robotics_assistant.nodes.dry_run_executor:main',
        ],
    },
)
