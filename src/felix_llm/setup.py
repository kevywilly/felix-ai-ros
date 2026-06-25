from glob import glob

from setuptools import find_packages, setup

package_name = 'felix_llm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # locations.yaml is symlinked back to source by --symlink-install, so
        # places taught at runtime persist in the repo (same trick as config.yml).
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Williams',
    maintainer_email='kevin.williams@sidusa.com',
    description='Natural-language "go to <room>" navigation via a local LLM.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent = felix_llm.agent_node:main',
            'teach = felix_llm.teach:main',
            'talk = felix_llm.talk:main',
            'mcp = felix_llm.mcp_server:main',
        ],
    },
)
