from setuptools import setup

setup(
    name='pai_safety',
    version='0.1.0',
    packages=['pai_safety'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_safety']),
                ('share/pai_safety', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'supervisor = pai_safety.supervisor:main',
        'human_sim = pai_safety.human_sim:main',
        'safety_check = pai_safety.safety_check:main',
    ]},
)
