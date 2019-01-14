import numpy as np
from open3d import *
from sys import argv
import matplotlib.pyplot as plt

def save_v(pcd):
    vis = Visualizer()
    vis.create_window()
    ctr = vis.get_view_control()
    vis.add_geometry(pcd)
    vis.run() # user changes the view and press "q" to terminate
    param = ctr.convert_to_pinhole_camera_parameters()
    trajectory = PinholeCameraTrajectory()
    trajectory.intrinsic = param[0]
    trajectory.extrinsic = Matrix4dVector([param[1]])
    write_pinhole_camera_trajectory("test.json", trajectory)
    vis.destroy_window()

def load_v(pcd):
    vis = Visualizer()
    vis.create_window()
    ctr = vis.get_view_control()
    trajectory = read_pinhole_camera_trajectory("test.json")
    vis.add_geometry(pcd)
    ctr.convert_from_pinhole_camera_parameters(
            trajectory.intrinsic, trajectory.extrinsic[0])
    vis.run()
    image = vis.capture_screen_float_buffer()
    plt.imshow(np.asarray(image))
    plt.savefig('figure.png')
    plt.show()
    vis.destroy_window()

pcd = read_point_cloud("disneymin.ply")
save_v(pcd)
load_v(pcd)

"""
mesh.

triangle_normals
triangles
vertex_colors
vertex_normals
vertices
"""

p1="mesh1_dis.ply"
p2="mesh1sub_dis.ply"
m1=read_triangle_mesh(p1)
m2=read_triangle_mesh(p2)
draw_geometries([m1])
draw_geometries([m2])

m=np.asarray(m1.triangles)
v=np.asarray(m1.vertices)
np.sum(np.isin(m,m[0]),axis=1)
np.where(np.sum(np.isin(m,m[0]),axis=1)==2)[0]
m[np.sum(np.isin(m,m[0]),axis=1)==2]


np.cross(v[m][:,1]-v[m][:,0],v[m][:,2]-v[m][:,0])/np.linalg.norm(np.cross(v[m][:,1]-v[m][:,0],v[m][:,2]-v[m][:,0]),axis=1)[:,np.newaxis]

r=np.pi/2
rz=np.matrix( (
    ( np.cos(r), np.sin(r), 0.),
    (-np.sin(r), np.cos(r), 0.),
    (     0.,     0., 1.)
) )

rx=np.matrix( (
    ( 1.,     0.,     0.),
    ( 0., np.cos(r), np.sin(r)),
    ( 0.,-np.sin(r), np.cos(r))
) )

ry=np.matrix( (
    ( np.cos(r), 0.,-np.sin(r)),
    (     0., 1.,     0.),
    ( np.sin(r), 0., np.cos(r))
) )

def rotx(vec,rad):
    return vec*np.matrix( (
    ( 1.,     0.,     0.),
    ( 0., np.cos(rad), np.sin(rad)),
    ( 0.,-1*np.sin(rad), np.cos(rad))
    ) )

def roty(vec,rad):
    return vec*np.matrix( (
    ( np.cos(rad), 0.,-1*np.sin(rad)),
    (     0., 1.,     0.),
    ( np.sin(rad), 0., np.cos(rad))
    ) )


m1.compute_triangle_normals()
n=np.asarray(m1.triangle_normals)

#角度の式
a = np.inner(xz, z)  #ベクトルの内積
b = np.linalg.norm(xz) * np.linalg.norm(z) #ベクトルの絶対値の積

c = a / b  # 角度の式 A・B = |A||B|cosθ からcosθを求める
theta = np.rad2deg(np.arccos(np.clip(c, -1.0, 1.0))) # clipで0以外の範囲に指定，arccosでラジアンに戻し，rad2degで度数表記にする

#a,b 2d vec
radab=np.arctan2(np.cross(a,b),np.dot(a,b))

radxz=np.arccos(np.clip(np.inner(xz, z) / np.linalg.norm(xz) * np.linalg.norm(z), -1.0, 1.0))

radyz=np.arccos(np.clip(np.inner(yz, z) / np.linalg.norm(yz) * np.linalg.norm(z), -1.0, 1.0))


base=v[m[0]]-v[m[0]][0]
base[1]

rotx(base[1],radyz)
roty(base[1],radxz)
