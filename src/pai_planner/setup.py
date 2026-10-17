from setuptools import setup

setup(
    name='pai_planner',
    version='0.1.0',
    packages=['pai_planner'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_planner']),
                ('share/pai_planner', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'executor = pai_planner.executor_node:main',
        'run_instruction = pai_planner.cli:main',
    ]},
)
