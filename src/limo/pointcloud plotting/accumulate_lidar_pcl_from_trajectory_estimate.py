import glob
import os

import numpy as np
import pykitti

import vtk_pointcloud as vtk_pcl

import argparse


def main(sequence, results_dir, kitti_data_dir, start_end_increment=None):
    poses_name = results_dir+"/{}.txt".format(sequence)

    if start_end_increment not is None:  # Get range of data.
        r_start = start_end_increment[0]
        r_end = start_end_increment[1]
        r_incr = start_end_increment[2]
        plot_range = range(r_start, r_end, r_incr)
        data = pykitti.odometry(kitti_data_dir, sequence, frames=plot_range)
    else:  # Get all.
        data = pykitti.odometry(kitti_data_dir, sequence)

    # Find all the Velodyne files
    velo_path = os.path.join(
        kitti_data_dir, 'sequences', sequence, 'velodyne_points', 'data', '*.bin')
    velo_files = sorted(glob.glob(velo_path))

    image_path = os.path.join(
        kitti_data_dir, 'sequences', sequence, 'image_2', 'data', '*.png')
    image_files = sorted(glob.glob(image_path))

    # Create the geometry of a point (the coordinate)
    pcl_colored = vtk_pcl.VtkPointCloud("rgb")
    pcl = vtk_pcl.VtkPointCloud("y")

    # load estimated poses
    estimate = np.loadtxt(poses_name)
    for i in plot_range:
        pose = np.concatenate((estimate[i, :].reshape(3, 4), np.array([[0, 0, 0, 1]])))

        scan = next(pykitti.utils.get_velo_scans([velo_files[i]]))
        img = next(pykitti.utils.get_images([image_files[i]], "cv2"))
        for poly_data_colored in scan:
            reflectance = poly_data_colored[3]
            if reflectance > 0.1:
                poly_data_colored[3] = 1
                p_cam0 = data.calib.T_cam0_velo.dot(poly_data_colored)
                p_world = pose.dot(p_cam0)

                pcl.add_point(p_world)

                # Get color data and add to visualizer.
                uv1 = data.calib.P_rect_20.dot(p_cam0)
                uv1 /= uv1[2]
                uv1 = np.floor(uv1).astype(int)

                if p_cam0[2] > 0.1 and 0 < uv1[0] < img.shape[1] - 1 and 0 < uv1[1] < img.shape[0] - 1:
                    color = img[uv1[1], uv1[0]]
                    # Add to visualizer.
                    pcl_colored.add_point(p_world, color)

        print("processed frame number {}".format(i))

    pcl.set_height(-2, 10, use_median=True)
    # write new PLY
    pcl_colored.write("/tmp/pcl_colored_{}_range_{}_{}_{}.ply".format(sequence, r_start, r_end, r_incr))
    print("wrote pcl_colored.ply to tmp")

    # write new PLY
    # pcl.z_max = pcl.z_min + 20
    pcl.write("/tmp/pcl_{}_range_{}_{}_{}.ply".format(sequence, r_start, r_end, r_incr))
    print("wrote pcl.ply to tmp")

    vtk_pcl.render(pcl)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--sequence", type=str, help="Sequence number.")
    parser.add_argument("--results", type=str, help="Directory where textfiles of results are stored.")
    parser.add_argument("--data", type=str, help="Path to kitti data (without parent folder of all sequences).")

    args = parser.parse_args()
    main(args.sequence, args.results, args.data)
