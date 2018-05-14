import subprocess
import os


def main():
    # Paths of the destination paths, KITTI_EVAL PARAM_SET is used for parameter tuning.
    # If unset it will just evaluate to ""
    dst_prepath = "/home/graeter/kitti_eval_tune_params/kitti_results_mono_lidar$1_$2/"
    # mkdir if not available
    os.makedirs(dst_prepath)

    # dump params
    with open(dst_prepath + "/params.yaml", "w") as file:
        file.write("depth_loss: %f\nreprojection_loss: %f" % (args.depth_loss, args.reprojection_loss))

    # Rosbags
    bagpath = "/media/graeter/data/odometry/kitti_bags/all_with_labels/"

    bagindex = [
        "00",
        "01",
        "02",
        "03",
        "04",
        "05",
        "06",
        "07",
        "08",
        "09",
        "10",
        "11",
        "12",
        "13",
        "14",
        "15",
        "16",
        "17",
        "18",
        "19",
        "20",
        "21"
    ]

    # _____ Run evaluation for all bags _____

    # source path
    os.system("source $HOME/workspaces/keyframe_ba/devel/setup.bash")

    # core
    processes = {}
    processes["core"] = subprocess.Popen(["roscore"])
    print("Started roscore")

    for bag in bagindex:
        print("Start eval sequ %s" % bag)
        # _____ Run main processes _____
        # start the odometry (rosnode) and close the newly created window after the process has been killed:
        comm_str = "roslaunch demo_keyframe_bundle_adjustment_meta kitti_standalone.launch | tee /tmp/log_$bag.txt"
        processes["main"] = subprocess.Popen(comm_str.split(" "))
        time.sleep(4.0)

        # reconfigure dump path by dynamic reconfigure
        dump_path_name = dst_prepath + bag + ".txt"
        comm_str = "rosrun dynamic_reconfigure dynparam set /mono_lidar dump_path " + dump_path_name
        processes["dyn_set0"] = subprocess.Popen(comm_str.split(" "))

        if args.depth_loss > 0.:
            print("set depth loss to %f" % args.depth_loss)
            comm_str = "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_depth_thres %f" % args.depth_loss)
            processes["dyn_set1"] = subprocess.Popen(comm_str.split(" "))

            if args.reprojection_loss > 0.:
                print("set reprojection loss to %f" % args.reprojection_loss)
            comm_str = "rosrun dynamic_reconfigure dynparam set /mono_lidar robust_loss_reprojection_thres %f" % args.reprojection_loss)
            processes["dyn_set2"] = subprocess.Popen(comm_str.split(" "))

            # dump tf stuff so that we can get all tf frames that we want afterwards
            bag_path_name = dst_prepath + bag + "_tf.bag"

            comm_str = "rosbag record /tf /tf_static /groundtruth_pose/pose /clock -o %s __name:=my_record" % (
                bag_path_name)
            processes["record"] = subprocess.Popen(comm_str.split(" "))

            # starte rosbag
            bagpath_full = bagpath + bag + ".bag"
            print("Start bag: %s" % bagpath_full

            # better run rhe rosbag sowly/safe to avoid the risk of frame skip (kitti evaluation failes)
            speed = 0.3
            comm_str = "rosbag play %s -r %f -d 6 --clock" % (bagpath_full, speed)
            processes["play"] = subprocess.Popen(comm_str.split(" "))

            # Kill monolidar gracefully to write all
            os.system("rosnode kill /mono_lidar")
            # Kill rosbag record via ros. Only killing window will leave it in active state.
            os.system("rosnode kill /my_record")
            time.sleep(2.0)

            # Kill stuff
            for name, p in processes:
                p.kill()

            time.sleep(5.0)

            print("done with sequence %s" % bag)

if __name__ == "__main__":
    main()
