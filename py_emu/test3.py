import numpy as np
from mayavi import mlab

class coo:
	x,y,z = 0.0,0.0,0.0
	def __init__(self, ix, iy, iz):
		self.x,self.y,self.z = ix,iy,iz
	def __str__(self):
		return "{}, {}, {}".format(self.x, self.y, self.z)
	def __mul__(self, val):
		return coo(self.x*val, self.y*val, self.z*val)
def dist(x1,y1,x2,y2):
	dx,dy = x1-x2, y1-y2
	return np.sqrt(dx*dx+dy*dy)

def cosab(a,b):
	absa = dist(a[0],a[1],0,0)
	absb = dist(b[0],b[1],0,0)
	return (a[0]*b[0]+a[1]*b[1])/absa/absb

def sinab(a,b):
	cos = cosab(a,b)
	return np.sqrt(1-cos*cos)

p = [[0,0],[2,0],[0,2]]

[x,y] = np.mgrid[-5.03:5:0.017, -5:5:0.017]

a01 = 0.5*(dist(x,y,p[0][0],p[0][1]) - dist(x,y,p[1][0],p[1][1]))
a01 = a01*a01
b01 = 1-a01
g1 = [2*(x-1)/a01, -2*y/b01]

a02 = 0.5*(dist(x,y,p[0][0],p[0][1]) - dist(x,y,p[2][0],p[2][1]))
a02 = a02*a02
b02 = 1-a02
g2 = [-2*x/b02, 2*(y-1)/a02]

a12 = 0.5*(dist(x,y,p[1][0],p[1][1]) - dist(x,y,p[2][0],p[2][1]))
a12 = a12*a12
b12 = 2-a12
g3 = [(x-y)/a12-(x+y-2)/b12, -(x-y)/a12-(x+y-2)/b12]

z = sinab(g1,g2) + sinab(g1,g3) + sinab(g2,g3)

mlab.mesh(x,y,z)
mlab.show()
