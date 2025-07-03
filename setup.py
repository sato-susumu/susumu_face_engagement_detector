from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'susumu_face_engagement_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='taro',
    maintainer_email='taro@example.com',
    description='Face engagement detection system with multiple nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_detection_node = susumu_face_engagement_detector.face_detection_node:main',
            'face_recognition_node = susumu_face_engagement_detector.face_recognition_node:main',
            'gaze_analysis_node = susumu_face_engagement_detector.gaze_analysis_node:main',
            'engagement_manager_node = susumu_face_engagement_detector.engagement_manager_node:main',
            'face_engagement_node = susumu_face_engagement_detector.face_engagement_node:main',
            'monitoring_node = susumu_face_engagement_detector.monitoring_node:main',
            'test_camera_node = susumu_face_engagement_detector.test_camera_node:main',
            'multi_node_executor = susumu_face_engagement_detector.multi_node_executor:main',
        ],
    },
)