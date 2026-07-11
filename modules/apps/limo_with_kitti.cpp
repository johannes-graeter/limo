#include <cstdlib>
#include <iostream>
#include <string>

#include "kitti_data_reader.h"
#include "rerun_vis.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <KITTI sequence dir> [--rerun spawn|connect[:<url>]|save:<path>|off]"
                  << std::endl;
        return EXIT_FAILURE;
    }
    const std::string kitti_path = argv[1];
    std::string rerun_mode = "spawn";
    for (int i = 2; i + 1 < argc; i += 2) {
        if (std::string(argv[i]) == "--rerun") {
            rerun_mode = argv[i + 1];
        }
    }

    rr_vis::RerunVisualizer vis("limo_with_kitti", rerun_mode);

    KittiDataReader reader(kitti_path);
    for (auto gen = reader.frames(); gen.next();) {
        const KittiFrame& frame = gen.value();
        vis.setFrame(static_cast<int64_t>(frame.index), 0);
        vis.logCamera("world/camera", frame.K, frame.image.cols, frame.image.rows);
        vis.logImage("world/camera/image", frame.image);
        vis.logPointCloud("world/lidar", *frame.cloud);
        std::cout << "Frame " << frame.index << ": image " << frame.image.cols << "x" << frame.image.rows
                  << ", cloud " << frame.cloud->size() << " points" << std::endl;
    }
    return EXIT_SUCCESS;
}
