#!/usr/bin/env python

# Helper script for auto-generating the spiral trajectory derivatives
# If you change this, also update spiral.cpp

from sympy import *
from sympy.utilities.codegen import codegen

t1, t2, theta1, theta2, radius = symbols('t1,t2,theta1,theta2,radius')
t = symbols('t')


a = theta1;
b = (radius - a) / theta2;

theta = (((t - t1) / (t2 - t1)) * (theta2 - theta1)) + theta1;

r = a + b*theta;

x = r*cos(theta)
y = r*sin(theta)


pos = Matrix([x, y, 0])
vel = diff(pos, t)
accel = diff(vel, t)


def print_vec(name, v):
	b_sym = symbols('b')
	r_sym = symbols('r')
	theta_sym = symbols('theta')
	v = v.subs(r, r_sym)
	v = v.subs(theta, theta_sym)
	v = v.subs(b, b_sym)


	# Generate taking only the computation line; the rhs of the equality; and removing the semicolon
	c = [codegen((name, v[i]), 'C', header=False)[0][1].split('\n')[6].split('=')[1].strip()[0:-1] for i in [0, 1, 2]]

	print(name + ' = Vector3d(' + c[0] + ', ' + c[1] + ', ' + c[2] + ');');


print_vec('s.position', pos) # TODO: '+ origin'
print_vec('s.velocity', vel)
print_vec('s.acceleration', accel)
