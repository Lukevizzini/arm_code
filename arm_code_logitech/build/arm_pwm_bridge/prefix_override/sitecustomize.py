import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/lukevizzini/logitech_arm_code/arm_code/arm_code_logitech/install/arm_pwm_bridge'
