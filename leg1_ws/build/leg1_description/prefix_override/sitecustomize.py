import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/vtcro/robogamescode/leg1_ws/install/leg1_description'
