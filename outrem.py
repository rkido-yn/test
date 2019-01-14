from open3d import *

def display_inlier_outlier(cloud, ind):
    inlier_cloud = select_down_sample(cloud, ind)
    outlier_cloud = select_down_sample(cloud, ind, invert=True)

    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    draw_geometries([inlier_cloud, outlier_cloud])


if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = read_point_cloud("gogh.ply")
    draw_geometries([pcd])

    print("Radius oulier removal")
    cl,ind = radius_outlier_removal(pcd,
                                    nb_points=16, radius=0.05)
    draw_geometries([cl])
    display_inlier_outlier(pcd, ind)
