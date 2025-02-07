import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yjh/junhyuk_project/turtlebot_ws/install/my_turtlebot_project'
