from setuptools import setup

package_name = 'hiwin_action_client'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Luis Tsai',
    maintainer_email='errrr0501done@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Hiwin_action_client = scripts.Hiwin_action_client:main'
        ],
    },
)
