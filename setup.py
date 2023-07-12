from setuptools import setup
import os
from glob import glob
package_name = 'obu'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='runze',
    maintainer_email='487844521@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'AbnormalInformationUpload  = obu.chassisAbnormalnformationUpload:main',
         'RealTimeInformationUpload  = obu.chassisRealTimeInformationUpload:main',
         'RegistratIonInformationReport = obu.chassisRegistrationInformationReport:main',
         'StatusInformationUpload = obu.chassisStatusInformationUpload:main',
         'SocketNode = obu.udp_interface_node:main',
         'service_client_node = obu.test:main'
        ],
    },
)