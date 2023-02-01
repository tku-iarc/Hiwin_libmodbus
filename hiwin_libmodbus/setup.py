from setuptools import setup

package_name = 'hiwin_libmodbus'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andy Chien',
    maintainer_email='N2107687J@ntu.edu.sg',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hiwin_api_test = scripts.Hiwin_API_test:main'
        ],
    },
)