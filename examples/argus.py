import subprocess
import sys
import os
import datetime
import re

# This is the executable (argus) file path. Change it correspondingly.
ARGUS = 'build/apps/argus-cloth'

python_path = os.path.dirname(os.path.realpath(__file__))
working_dir = os.path.dirname(python_path)

# Commands
commands = ['simulate', 'simulateoffline', 'resume', 'resumeoffline', 'replay']
COMMAND_SIMULATE = 0
COMMAND_SIMULATE_OFFLINE = 1
COMMAND_RESUME = 2
COMMAND_RESUME_OFFLINE = 3
COMMAND_REPLAY = 4

# Examples
examples = ['box_and_cone', 'belt_high_tension', 'cards_high_friction', 'cards_low_friction', 'drag_cloth', 
	'character/Arabesque/KyraArabesque-sixtiesdress-0-0',
	'character/Arabesque/KyraArabesque-sixtiesdress-0-3',
	'character/Clubbing/KyraClubbing-sixtiesdress-0-0',
	'character/Clubbing/KyraClubbing-sixtiesdress-0-1',
	'character/Clubbing/KyraClubbing-sixtiesdress-0-3',
	'character/Twist/KyraTwist-batdress-0-0',
	'character/Twist/KyraTwist-batdress-0-1',
	'character/Twist/KyraTwist-batdress-0-3',
	'character/Twist/KyraTwist-batdress-0-6',
	'character/Shawl/KyraShawl-shawl-0-3',
	'character/Shawl/KyraShawl-shawl-0-6',
	'character/HipHop/KyraHipHop-hoodie-0-0',
	'character/HipHop/KyraHipHop-hoodie-0-3']
mus = [0.2, 0.4, 0.6, 0.2, 0.6, 0, 0.3, 0, 0.1, 0.3, 0, 0.1, 0.3, 0.6, 0.3, 0.6, 0, 0.3]

EXAMPLE_BOX_AND_CONE = 0
EXAMPLE_BELT_HIGH_TENSION = 1
EXAMPLE_CARDS_HIGH_FRICTION = 2
EXAMPLE_CARDS_LOW_FRICTION = 3
EXAMPLE_DRAG_CLOTH = 4
EXAMPLE_ARABESQUE_0 = 5
EXAMPLE_ARABESQUE_03 = 6
EXAMPLE_CLUBBING_0 = 7
EXAMPLE_CLUBBING_01 = 8
EXAMPLE_CLUBBING_03 = 9
EXAMPLE_TWIST_0 = 10
EXAMPLE_TWIST_01 = 11
EXAMPLE_TWIST_03 = 12
EXAMPLE_TWIST_06 = 13
EXAMPLE_SHAWL_03 = 14
EXAMPLE_SHAWL_06 = 15
EXAMPLE_HIPHOP_0 = 16
EXAMPLE_HIPHOP_03 = 17

objects_path_prefix = os.path.join('output', 'objects')
images_path_prefix = os.path.join('output', 'images')
config_path_prefix = 'conf'


def data_synthesis(log_file, output_file, mu):
	save_idx = [1, 2, 4, 5, 6, 7, 8, 10, 12, 14, 15];
	data = [0]*len(save_idx)
	with open(log_file) as lf:
		count = 0
		while True:
			line = lf.readline()
			if not line:
				break
			segs = line.split()
			for i, idx in enumerate(save_idx):
				data[i] += float(segs[idx])
			count += 1
		data = [x/count for x in data]
		
	of = open(output_file, 'a')
	of.write('{}\t'.format(mu))
	for x in data:
		of.write('{}\t'.format(x))
	of.write('\n')

def simulate(display = True, demo = EXAMPLE_BOX_AND_CONE, save_objects = False, performance_file = '', objects_path_given = ''):
	os.chdir(working_dir)	# switch to the root of the project
	if display:
		cmd = COMMAND_SIMULATE
	else:
		cmd = COMMAND_SIMULATE_OFFLINE
	demo_name = examples[demo]
	config = os.path.join(config_path_prefix, '{}.json'.format(demo_name))
	print config
	if save_objects:
		time_now = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
		if len(objects_path_given) > 0:
			objects_path = os.path.join(objects_path_given, '{}_{}'.format(demo_name, time_now))
		else:
			objects_path = os.path.join(objects_path_prefix, demo_name, time_now)
		if not os.path.exists(objects_path):
			os.makedirs(objects_path)
	else:
		objects_path = ''
	args = [ARGUS, commands[cmd], config, objects_path]
	subprocess.call(args)
	if len(performance_file) > 0 and save_objects:
		log_file = os.path.join(objects_path, 'log.txt')
		data_synthesis(log_file, performance_file, mus[demo])

def get_run_frames(objects_path):
	frame = -1
	for file in os.listdir(objects_path):
		obj_file = re.findall('(\\d{6})_\\d{2}.obj', file)
		if obj_file:
			obj_frame = int(obj_file[0])
			if frame < obj_frame:
				frame = obj_frame
	return frame

def no_objects_found(demo_name):
	print 'No saved objects of example {} found'.format(demo_name)
	exit(-1)

def find_existing_objects_path(demo_name):
	objects_path_dir = os.path.join(objects_path_prefix, demo_name)
	if not os.path.exists(objects_path_dir):
		no_objects_found(demo_name)
	latest_date = datetime.datetime.min
	run_dirs = os.listdir(objects_path_dir)
	if len(run_dirs) == 0:
		no_objects_found(demo_name)
	for run_dir in run_dirs:
		date_str = re.findall('(\\d{4}(-\\d{2}){5})', run_dir)
		if not date_str:
			continue
		run_date = datetime.datetime.strptime(run_dir, '%Y-%m-%d-%H-%M-%S')
		if latest_date < run_date:
			latest_date = run_date
	if latest_date == datetime.datetime.min:
		no_objects_found(demo_name)
	latest_dir = latest_date.strftime('%Y-%m-%d-%H-%M-%S')
	return os.path.join(objects_path_dir, latest_dir)


def resume(display = True, demo = EXAMPLE_BOX_AND_CONE, frame = -1):
	os.chdir(working_dir)
	if display:
		cmd = COMMAND_RESUME
	else:
		cmd = COMMAND_RESUME_OFFLINE
	demo_name = examples[demo]
	objects_path = find_existing_objects_path(demo_name)
	if frame == -1: # start from the last frame
		frame = get_run_frames(objects_path)
	if frame < 0:
		no_objects_found(demo_name)
	args = [ARGUS, commands[cmd], objects_path, str(frame)]
	subprocess.call(args)

def replay(demo = EXAMPLE_BOX_AND_CONE, save_images=False):
	os.chdir(working_dir)
	cmd = COMMAND_REPLAY
	demo_name = examples[demo]
	objects_path = find_existing_objects_path(demo_name)
	if save_images:
		time_now = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
		images_path = os.path.join(images_path_prefix, demo_name, time_now)
		if not os.path.exists(images_path):
			os.makedirs(images_path)
	else:
		images_path = ''
	args = [ARGUS, commands[cmd], objects_path, images_path]
	subprocess.call(args)