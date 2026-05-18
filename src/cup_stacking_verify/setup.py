from setuptools import find_packages, setup

package_name = 'cup_stacking_verify'

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
    maintainer='ssu',
    maintainer_email='ssu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'verifier = cup_stacking_verify.verifier_node:main',
            'test_publisher = cup_stacking_verify.test_pub:main', # 이 줄이 반드시 있어야 함!
        ],
    },
)