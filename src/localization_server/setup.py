import os
from glob import glob



(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'), glob('config/*')),
(os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),

entry_points={
    'console_scripts': [
        'initial_pose_pub = localization_server.initial_pose_pub:main',
    ],
},
