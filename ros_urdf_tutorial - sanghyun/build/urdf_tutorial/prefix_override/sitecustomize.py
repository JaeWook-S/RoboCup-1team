import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sh0819/ros_urdf_tut/install/urdf_tutorial'
