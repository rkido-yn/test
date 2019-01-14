# -*- coding: utf-8 -*-
import numpy as np
from open3d import *

def create_sample():
  x=np.array(np.linspace(-1,1,num=10).tolist()*10)
  y=np.repeat(np.linspace(-1,1,num=10),10)
  z=-x**2+1
  d=np.stack([x,y,z],axis=0).T
  d=np.append(d,[[0,0,1.3]],axis=0)
  pcd=PointCloud()
  pcd.points=Vector3dVector(d)

  return pcd

def project(cent,n,p0):
  return p0-n*np.dot(p0-cent,n)


def Covariance(p,idx,cent):
  if len(p) == 0:
    return 0
  covamat=np.zeros((3,3))
  pt=np.zeros((4,1))
  for i in len(p):
    pt[0]=p[i][0]-cent[0]
    pt[1]=p[i][1]-cent[1]
    pt[2]=p[i][2]-cent[2]

    covamat[1][1]+=pt[1]*pt[1]
    covamat[1][2]+=pt[1]*pt[2]
    covamat[2][2]+=pt[2]*pt[2]
    pt*=pt[0]
    covamat[0][0]+=pt[0]
    covamat[0][1]+=pt[1]
    covamat[0][2]+=pt[2]
    return covamat
#np.covでできる？

#def main():
#pcd = read_point_cloud("a.0.ply")
pcd = create_sample()
draw_geometries([pcd])
ps0=np.array(pcd.points)#
ps=np.asarray(pcd.points)#
pcd_tree = KDTreeFlann(pcd)

"""
[k, idx, _] = pcd_tree.search_radius_vector_3d(pts0[-1], 0.3)
pcd.colors=Vector3dVector(np.array([[0,0,0]]*len(pts0)))
np.asarray(pcd.colors)[idx, :] = [1, 0, 0]
"""

for i,p0 in enumerate(ps0):
  [k, idx, _] = pcd_tree.search_radius_vector_3d(p0, 0.7)
  #print p,
  if k<3:continue
  #cent=np.sum(ps[idx],axis=0)/k
  cent=np.mean(ps0[idx],axis=0)
  #cent=np.append(cent,1)
  #covamat=Covariance(p,idx,cent)
  covamat=np.cov(ps0[idx].T,bias=True)
  w, v =np.linalg.eig(covamat)
  ps[i]=project(cent,v[:,np.argmin(w)],p0)
  print(":{}".format(ps[i]))
print(ps)
draw_geometries([pcd])

#if __name__ == '__main__':
#    main()
