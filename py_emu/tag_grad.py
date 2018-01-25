import numpy as np
from mayavi import mlab

class coordinate:
	x, y, z = 0.0, 0.0, 0.0
	def __init__(self, ix, iy, iz):
		self.x, self.y, self.z = ix,iy,iz
	def __add__(self, val):
		return coordinate(self.x+val.x,self.y+val.y,self.z+val.z)
	def __sub__(self, val):
		return coordinate(self.x-val.x,self.y-val.y,self.z-val.z)
	def __mul__(self, val):
		return coordinate(self.x*val, self.y*val, self.z*val)

def dist(p1, p2):
	dx, dy, dz = p1.x-p2.x, p1.y-p2.y, p1.z-p2.z
	tmp = np.sqrt(dx*dx+dy*dy+dz*dz)
	if tmp < 0.000001:
		tmp = 0.000001
	return tmp

def grad(p1, p2, p):
	dist1 = dist(p, p1)
	dist2 = dist(p, p2)
	return coordinate(
		(p.x-p1.x)/dist1-(p.x-p2.x)/dist2,
		(p.y-p1.y)/dist1-(p.y-p2.y)/dist2,
		(p.z-p1.z)/dist1-(p.z-p2.z)/dist2)

def inte_grad(p1, p2, p):
	g = grad(p1,p2,p)
	d = dist(g, coordinate(0,0,0))
	g.x = g.x/d
	g.y = g.y/d
	g.z = g.z/d
	return g

# anchors' location
p1 = coordinate(0,	0,	0)
p2 = coordinate(2.0, 	0,	0)
p3 = coordinate(1, 	1.5,	0)
p4 = coordinate(1, 	0,	2)

# target's location
pt = coordinate(1,	0.7,	1)
d = [dist(pt,p1), dist(pt,p2), dist(pt,p3), dist(pt,p4)]
dd = [d[1]-d[0], d[1]-d[2], d[1]-d[3]]
# start point
ps = coordinate(-2.0,	-3,	0)

# gradient decent
step = 0.4
x, y, z = [], [], []
for i in range(500):
	x.append(ps.x)
	y.append(ps.y)
	z.append(ps.z)
	sd = [dist(ps,p1), dist(ps,p2), dist(ps,p3), dist(ps,p4)]
	dsd = [sd[1]-sd[0], sd[1]-sd[2], sd[1]-sd[3]]
	delta = [dd[0]-dsd[0], dd[1]-dsd[1], dd[2]-dsd[2]]
	e = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2]
	if e < 0.0002:
		print i, ' iterations'
		break
	# grad
	g1 = inte_grad(p2, p1, ps)*step*delta[0]
	g2 = inte_grad(p2, p3, ps)*step*delta[1]
	g3 = inte_grad(p2, p4, ps)*step*delta[2]
	ps = ps + g1 + g2 + g3

print ps.x, ps.y, ps.z
mlab.points3d([p1.x,p2.x,p3.x,p4.x],
              [p1.y,p2.y,p3.y,p4.y],
              [p1.z,p2.z,p3.z,p4.z],
              color=(1.0,0,0), scale_factor=0.2)
mlab.points3d([pt.x],[pt.y],[pt.z], color=(0,1.0,0), scale_factor=0.05)
mlab.points3d(x,y,z, scale_factor=0.05)
mlab.show()
