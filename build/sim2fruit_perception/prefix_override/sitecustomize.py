import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ronyfaz/sim2fruit_ws/install/sim2fruit_perception'
