import os
import subprocess
import math

python_path = os.path.dirname(os.path.realpath(__file__))
working_dir = os.path.dirname(python_path)

data_dir = 'output/figures/fig_5'
simulated_data_file = 'mass_center_simulated.txt'
analytical_data_file = 'mass_center_analytical.txt'

simulated_file_path = os.path.join(data_dir, simulated_data_file)
os.chdir(working_dir)
if not os.path.exists(data_dir):
	os.makedirs(data_dir)

of = open(simulated_file_path, 'w')
of.write('t\tv\th\n')
of.close()

subprocess.call(['build/apps/argus-cloth', 'simulateoffline', 'conf/inclined_plane.json'])

def dot(a, b):
	return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]

def multiply(v, e):
	return [v[0]*e, v[1]*e, v[2]*e]

def add(a, b):
	return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]

def minus(a, b):
	return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]

def norm(v):
	return math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])

dh = [0, 0, -0.521562]; # Height (vertical distance) between the cloth and the plane.
g = [0, 0, -9.8];	# Gravity acceleration

v0 = [0, 0, -0.2];	# The initial velocity
h0 = -0.2108	# The initial height
# Compute the time when the cloth hits the inclined plane (t_hit = 0.3065)
t_hit = -(math.sqrt(dot(v0, v0) + 2*dot(g, dh)) + v0[2])/g[2]

n = [0.422592, 0.0010409, 0.906319]	# Normal direction of the plane (unit vector).
vertical = [0, 0, 1]	#vertical direction
mu = 0.2 # The friction coefficient
g_n = multiply(n, dot(g, n)) # The component of g in the normal direction.
g_t = minus(g, g_n)	# The component of g in the tangential plane.
f = multiply(g_t, (norm(g_t) - mu*norm(g_n))/norm(g_t)) # The net acceleration (g_t - acceleration by friction).

# The velocity when the cloth hit the plane. v_hit = v0 + g*t_hit
v_hit = add(v0, multiply(g, t_hit))
# The component of the velocity in the normal direction. Projection of v_hit to n.
v_n = multiply(n, dot(v_hit, n))
# The component of the velocity in the plane.
v_t = minus(v_hit, v_n)
# The proportion of velocity in the plane remaining after the hit
v_t_remaining = (norm(v_t) - mu*norm(v_n))/norm(v_t)
# The actual velocity in the plane after the hit
v_t_new = multiply(v_t, v_t_remaining)

fv = multiply(vertical, dot(f, vertical))	# Net acceleration in the vertical direction (after the hit)
vv = multiply(vertical, dot(v_t_new, vertical))	# Velocity in the vertical direction (after the hit)

analytical_file_path = os.path.join(data_dir, analytical_data_file)
of = open(analytical_file_path, 'w')
of.write('t\tv\th\n')

t = 0.0
while t <= 1.0:
	if t <= t_hit: # Before hitting the plane, the motion is free fall.
		v = add(v0, multiply(g, t))	# v = v0 + g*t
		dis = add(multiply(v0, t), multiply(g, 0.5*t*t)) # dis = v0*t + .5*g*t^2
		h = h0 + dis[2]
	else:
		# Time elapsed after the hit
		tnew = t - t_hit
		v = add(v_t_new, multiply(f, tnew)) # v = v_t_new + f*tnew
		dis = add(multiply(vv, tnew), multiply(fv, 0.5*tnew*tnew)) # dis = vv*tnew + 0.5*fv*tnew^2
		h = h0 + dh[2] + dis[2]

	print('{}\t{}'.format(norm(v), h))
	of.write('{}\t{}\t{}\n'.format(t, norm(v), h))
	t += 1e-3

of.close()


subprocess.call(['pdflatex', 'figures/fig_5.tex'])