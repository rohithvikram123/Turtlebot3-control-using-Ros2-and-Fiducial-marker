import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rohithvikram/C++learning/Turtlebot3-control-using-Ros2-and-Fiducial-marker/install/tb3_gazebo_spawner'
