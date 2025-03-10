from setuptools import setup

package_name = 'my_voice'

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
    maintainer='jmj',
    maintainer_email='jmj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Pub = my_voice.Pub:main',
            'Sub = my_voice.Sub:main',
            'stt = my_voice.stt:main',
            'button_pressed = my_voice.button_pressed:main',
        ],
    },
)
