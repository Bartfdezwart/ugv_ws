from setuptools import find_packages, setup

package_name = 'ugv_custom'

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
    maintainer='bart',
    maintainer_email='bart@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drive_and_blink = ugv_custom.drive_and_blink:main',
            'detect_lines_canny = ugv_custom.detect_lines_canny:main',
            'detect_lines_custom = ugv_custom.detect_lines_custom:main',
            'detect_lines_select = ugv_custom.detect_lines_select:main',
            'detect_markers = ugv_custom.detect_markers:main',
            'follow_line = ugv_custom.follow_line:main',
        ],
    },
)
