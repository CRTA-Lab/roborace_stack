from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CRTA-racer',
    maintainer_email='cirkvencic.karlo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'purePursuit_wp = pure_pursuit.purePursuit_wp:main',
            'purePursuit_rrt = pure_pursuit.purePursuit_rrt:main',
            'stanley = pure_pursuit.stanley:main',
            'purePursuit_opti = pure_pursuit.purePursuit_opti:main',
            'pid = pure_pursuit.PID:main',
            'bangbang = pure_pursuit.bangbang:main'
        ],
    },
)
