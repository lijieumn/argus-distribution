import os
current_py = os.path.dirname(os.path.realpath(__file__))
python_path = os.path.abspath(os.path.join(current_py, os.pardir))
import sys
sys.path.append(python_path)
import argus

_example = argus.EXAMPLE_CLUBBING_01

argus_interface = os.path.join(python_path, 'argus_interface.py')

os.system('python {} -e {}'.format(argus_interface, _example))