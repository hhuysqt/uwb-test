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

p = [[-3,-3],[3,-3],[-3,3]]
nr_anc = len(p)

[ax,ay] = [-1.0, .0]
aim_d = []
for i in range(nr_anc):
	aim_d.append(dist(ax,ay,p[i][0],p[i][1]))
	if i > 2:
		aim_d[i] = aim_d[i] + 0.01
d = []
for i in range(1, nr_anc):
	d.append(aim_d[0]-aim_d[i])

[x,y] = np.mgrid[-100.27:100:0.5, -100.27:100:0.5]
cur_d = []
for i in range(nr_anc):
	cur_d.append(dist(x,y,p[i][0],p[i][1]))
delt = []
for i in range(1, nr_anc):
	delt.append(cur_d[0]-cur_d[i] - d[i-1])
z = delt[0]*delt[0]
for i in range(1, nr_anc-1):
	z = z + delt[i]*delt[i]

mlab.points3d([p[0][0],p[1][0],p[2][0], ax],[p[0][1],p[1][1],p[2][1], ay],[0,0,0,0])
mlab.mesh(x,y,z)
# mlab.show()

grad = [delt[0]*((x-p[0][0])/cur_d[0] - (x-p[1][0])/cur_d[1]),
        delt[0]*((y-p[0][1])/cur_d[0] - (y-p[1][1])/cur_d[1])]
for i in range(1, nr_anc-1):
	grad[0] = grad[0] + delt[i]*((x-p[0][0])/cur_d[0] - (x-p[i+1][0])/cur_d[i+1])
	grad[1] = grad[1] + delt[i]*((y-p[0][1])/cur_d[0] - (y-p[i+1][1])/cur_d[i+1])
z = dist(grad[0],grad[1],0,0) - 10
#mlab.mesh(x,y,z)
mlab.show()
print z[0][0], z[200][200]
