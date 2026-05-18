import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eunwoosong/Projects/vision-node/install/cup_stacking_verify'
