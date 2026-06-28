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
            'head_pose_node = susumu_face_engagement_detector.head_pose_node:main',
            'expression_node = susumu_face_engagement_detector.expression_node:main',
            'engagement_node = susumu_face_engagement_detector.engagement_node:main',
            'face_engagement_video_demo = eval.video_demo:main',
            'chokepoint_gt_demo = eval.chokepoint_demo:main',
        ],
    },
)
