#include <iostream>
#include <feature_tracking_core/tracker.h>
#include <keyframe_bundle_adjustment/bundle_adjuster_keyframes.hpp>
#include "kitti_data_reader.h"
#include "rerun_vis.h"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <KITTI sequence base path>" << std::endl;
        return 1;
    }

    rr_vis::RerunVisualizer rec("LIMO with KITTI");

    std::string kitti_path = argv[1];
    KittiDataReader reader(kitti_path);
    size_t count = 0;
    rerun::init();
    for (auto gen = reader.frames(); gen.next(); ) {
        const KittiFrame& frame = gen.value();
        std::cout << "Frame " << frame.index << ": Image size = " << frame.image.cols << "x" << frame.image.rows
                  << ", PointCloud size = " << frame.cloud->size() << std::endl;
        rec.log(rec, "KITTI Image 3D", frame.image, frame.K, frame.Tr);
        rec.log(rec, "KITTI PointCloud", frame.cloud);
        // Example: show image (uncomment to enable)
        // cv::imshow("KITTI Image", frame.image);
        // if (cv::waitKey(1) == 27) break;
        if (++count > 5) break; // Only process first 5 frames for demo
    }
    return 0;
}
