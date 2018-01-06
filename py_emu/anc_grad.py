import numpy as np
from mayavi import mlab

class coordinate:
	x, y, z = 0.0, 0.0, 0.0
	def __init__(self, ix, iy, iz):
		self.x, self.y, self.z = ix,iy,iz
	def __str__(self):
		return "{}, {}, {}".format(self.x, self.y, self.z)
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

def int_grad(p1, p2):
	return (p2-p1) * (1.0/dist(p1,p2))

def adj_val(a):
	return a + a/(1+a*a)

# anchors' location
p = [
    coordinate(0,	0,	0),
    coordinate(3.0, 	0,	0),
    coordinate(0,	4.0,	0),
    coordinate(8, 	0,	4),
    coordinate(0, 	-8,	4),
    coordinate(0, 	-8,	-7.34)
  ]
nr_anchor = len(p)

# distances between anchors
d_anc = [
    [0],
    [dist(p[1],p[0])],
    [dist(p[2],p[0]), dist(p[2],p[1])],
  ]
is_okay = []
for i in range(3, nr_anchor):
	dtmp = []
	for j in range(0, i):
		dtmp.append(dist(p[i],p[j]))
	d_anc.append(dtmp)
	is_okay.append(0)
	print d_anc[i]

# start points: the first 3 points are pre-calculated
c = d_anc[1][0]
b = d_anc[2][0]
a = d_anc[2][1]
x = (b*b+c*c-a*a)/2/c
y = np.sqrt(b*b - x*x)
sp = [
    coordinate(0,	0,	0),
    coordinate(c,	0,	0),
    coordinate(x, 	y,	0),
  ]

x,y,z = [],[],[]
for i in range(3, nr_anchor):
	sp.append(coordinate(0, 	0,	0.0001))
	x.append([]), y.append([]), z.append([])

# gradient decent
step = 0.1
for i in range(1000):
	# current distances
	es = 0
	for j in range(3, nr_anchor):
		if is_okay[j-3] == 1:
			continue
		s_anc = []
		for k in range(0, j):
			s_anc.append(dist(sp[j],sp[k]))
		dists = np.array(d_anc[j]) - np.array(s_anc)
		e = (dists*dists).sum()
		if e < 0.0002:
			print d_anc[j]
			print s_anc
			print i, ' iterations on anchor ', j
			is_okay[j-3] = 1
			continue
		es = es + e
		# grad
		tmp = coordinate(0,0,0)
		for k in range(0,j):
			grad = int_grad(sp[k], sp[j])
			tmp = tmp + grad * adj_val(dists[k] * step)
		sp[j] = sp[j] + tmp
		x[j-3].append(sp[j].x)
		y[j-3].append(sp[j].y)
		z[j-3].append(sp[j].z)
	if es < 0.0002:
		print 'okay'
		break
	

print sp[3]
mlab.points3d([p[0].x,p[1].x,p[2].x],
              [p[0].y,p[1].y,p[2].y],
              [p[0].z,p[1].z,p[2].z],
              color=(1.0,0,0), scale_factor=0.2)
for i in range(3, nr_anchor):
	mlab.points3d([p[i].x],[p[i].y],[p[i].z], color=(0,1.0,0), scale_factor=0.05)
	mlab.points3d(x[i-3],y[i-3],z[i-3], scale_factor=0.05)
mlab.show()
