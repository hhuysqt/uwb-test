import numpy as np
from mayavi import mlab

def dist(x1,y1,z1, x2,y2,z2):
	dx,dy,dz = x1-x2,y1-y2,z1-z2
	return np.sqrt(dx*dx + dy*dy + dz*dz)

p = [[-2,0,0], [2,0,0], [0,2,0]]

x,y,z = [],[],[]
sx,sy,sz = -2.5,-2.5,-5

for i in range(100):
	for j in range(100):
		for k in range(100):
			cx,cy,cz = sx+i*0.05, sy+j*0.05, sz+k*0.1
			d1 = dist(cx,cy,cz, p[0][0],p[0][1],p[0][2])
			d2 = dist(cx,cy,cz, p[1][0],p[1][1],p[1][2])
			d3 = dist(cx,cy,cz, p[2][0],p[2][1],p[2][2])
			d21,d31 = d2-d1-0.8,d3-d1-0.7
			if  d21*d21 < 0.001 and d31*d31 < 0.001 :
				x.append(cx), y.append(cy), z.append(cz)

mlab.points3d([p[0][0],p[1][0],p[2][0]],
              [p[0][1],p[1][1],p[2][1]],
              [p[0][2],p[1][2],p[2][2]],
              color=(1.0,0,0), scale_factor=0.2)
mlab.points3d(x,y,z,scale_factor=0.05, color=(0,0,1))
mlab.show()
