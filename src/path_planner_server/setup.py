import os
from glob import glob
        
        
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
(os.path.join('share', package_name, 'config'), glob('config/*')),
