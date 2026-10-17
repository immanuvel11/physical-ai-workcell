from setuptools import setup

setup(
    name='pai_skills',
    version='0.1.0',
    packages=['pai_skills'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/pai_skills']),
                ('share/pai_skills', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'skill_server = pai_skills.skill_server:main',
        'skills_check = pai_skills.skills_check:main',
    ]},
)
