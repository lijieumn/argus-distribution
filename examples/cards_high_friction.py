import os
import argus

_example = argus.EXAMPLE_CARDS_HIGH_FRICTION

python_path = os.path.dirname(os.path.realpath(__file__))
argus_interface = os.path.join(python_path, 'argus_interface.py')

os.system('python {} -e {}'.format(argus_interface, _example))