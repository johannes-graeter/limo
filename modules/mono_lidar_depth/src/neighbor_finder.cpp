#include "mono_lidar_depth/neighbor_finder.hpp"

#include <algorithm>

namespace mono_lidar {

PixelNeighborFinder::PixelNeighborFinder(int img_width, int img_height, int pixel_search_width, int pixel_search_height)
        : img_width_(img_width), img_height_(img_height), pixel_search_width_(pixel_search_width),
          pixel_search_height_(pixel_search_height) {
    pixel_to_visible_index_.resize(img_width_, img_height_);
}

void PixelNeighborFinder::initializeProjection(const ProjectedPointcloud& points) {
    pixel_to_visible_index_.setConstant(kNoPoint);

    const auto point_count = static_cast<int>(points.visible_image_cs.cols());
    for (int i = 0; i < point_count; ++i) {
        const auto x_img = static_cast<int>(points.visible_image_cs(0, i));
        const auto y_img = static_cast<int>(points.visible_image_cs(1, i));

        // First point in front of the camera wins if several project to the same pixel.
        if (pixel_to_visible_index_(x_img, y_img) == kNoPoint && points.visiblePointCameraCs(i).z() > 0.) {
            pixel_to_visible_index_(x_img, y_img) = i;
        }
    }
}

std::vector<int> PixelNeighborFinder::findNeighborIndices(const Eigen::Vector2d& feature_image_cs,
                                                          float scale_width,
                                                          float scale_height) const {
    const double half_size_x = pixel_search_width_ * 0.5 * scale_width;
    const double half_size_y = pixel_search_height_ * 0.5 * scale_height;

    const auto left = static_cast<int>(std::max(feature_image_cs.x() - half_size_x, 0.));
    const auto right = static_cast<int>(std::min(feature_image_cs.x() + half_size_x, img_width_ - 1.));
    const auto top = static_cast<int>(std::max(feature_image_cs.y() - half_size_y, 0.));
    const auto bottom = static_cast<int>(std::min(feature_image_cs.y() + half_size_y, img_height_ - 1.));

    std::vector<int> visible_indices;
    for (int y = top; y <= bottom; ++y) {
        for (int x = left; x <= right; ++x) {
            if (const int index = pixel_to_visible_index_(x, y); index != kNoPoint) {
                visible_indices.push_back(index);
            }
        }
    }
    return visible_indices;
}

VecOfVec3d PixelNeighborFinder::collectPoints(const ProjectedPointcloud& points,
                                              const std::vector<int>& visible_indices) {
    VecOfVec3d neighbors;
    neighbors.reserve(visible_indices.size());
    for (const int index : visible_indices) {
        neighbors.push_back(points.visiblePointCameraCs(index));
    }
    return neighbors;
}

} // namespace mono_lidar
