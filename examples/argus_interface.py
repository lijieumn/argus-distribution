import argus
import argparse

_example = argus.EXAMPLE_BELT_HIGH_TENSION
_cmd = 0
_display = True
_save = True
_performancefile = ''
_outputpath = ''

parser = argparse.ArgumentParser(description='Run Argus demos with Python')
parser.add_argument('-c', metavar='command', type=int, default=_cmd, help='Argus command (default is simulate). Options: 0 (simulate), 1 (resume), 2 (replay)')
parser.add_argument('-e', required=True, metavar='example', type=int, default=_example, help='Examples provided (required). Options: 0 (box_and_pin), 1 (belt_high_tension), 2 (cards_high_friction), 3 (cards_low_friction), 4 (drag_cloth), 5-6 (arabesque with mu = 0, 0.3), 7-9 (clubbing with mu = 0, 0.1, 0.3), 10-13 (twist with mu = 0, 0.1, 0.3, 0.6), 14-15 (shawl with mu = 0.3, 0.6), 16-17 (hiphop with mu = 0, 0.3)')
parser.add_argument('-d', metavar='display', type=int, default=_display, help='Whether to run the simulation online or offline (default is offline). 0 for offline, others for online.')
parser.add_argument('-s', metavar='save', type=int, default=_save, help='Whether to save the output files (default is to save the outputs). This is only used for simulate and replay: in simulate command, it means whether to save object files, in replay command, it means whether to save images. 0 for not save and others for save')
parser.add_argument('-p', metavar='performancefile', type=str, default=_performancefile, help='The file to write the average performance data into, including running time, intersections, number of iterations, errors, etc.')
parser.add_argument('-o', metavar='outputpath', type=str, default=_outputpath, help='The path to save the output (the object files when simulating). If this is not specified, the object files will be saved under output directory at the root location of the project.')

args = parser.parse_args()

cmd = args.c
example = args.e
display = args.d
save = args.s
performance_file = args.p
outputpath = args.o

if cmd == 0:
	argus.simulate(display = display, demo = example, save_objects = save, performance_file = performance_file, objects_path_given = outputpath)
elif cmd == 1:
	argus.resume(display = display, demo = example)
elif cmd == 2:
	argus.replay(demo = example, save_images = save)
else:
	print('Invalid command code {}. Options: 0 (simulate), 1 (resume), 2 (replay)'.format(cmd))
