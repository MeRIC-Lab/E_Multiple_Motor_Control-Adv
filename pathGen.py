from math import *

def TestPath(omega, R, t):
    pos = [R*sin(omega*t)+2048, R*cos(omega*t)+2048]
    vel = [R*omega*cos(omega*t), -1.0*R*omega*sin(omega*t)]

    return pos, vel
