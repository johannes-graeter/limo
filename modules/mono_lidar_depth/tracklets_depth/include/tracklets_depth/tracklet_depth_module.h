#pragma once

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <matches_msg_ros/MatchesMsg.h>
#include <feature_tracking_core/tracklet.h>
#include <matches_msg_depth_ros/MatchesMsg.h>
#include <monolidar_fusion/camera_pinhole.h>
#include <monolidar_fusion/DepthEstimator.h>
#include <monolidar_fusion/DepthEstimatorParameters.h>
#include <monolidar_fusion/DepthCalculationStatistics.h>
#include <monolidar_fusion/RansacPlane.h>
#include <tracklets_depth/TempTrackletFrame.h>
#include <tracklets_depth/parameters.h>

namespace tracklets_depth {

class TrackletDepthModule {
  using Point = pcl::PointXYZI;
  using Cloud = pcl::PointCloud<Point>;
  using CameraInfo = sensor_msgs::CameraInfo;
  using Tracklet = matches_msg_ros::MatchesMsg;
  using TypeTrackletKey = u_int64_t;
  using StampType = std_msgs::Header::_stamp_type;

 public:
  TrackletDepthModule(
      const Mono_Lidar::DepthEstimatorParameters& depth_estimator_parameters):
        depth_estimator_parameters_(depth_estimator_parameters) {
    _cloud_last_frame = nullptr;
    // ransac plane will be initialializedin depth estimator
    groundPlaneLast_ = nullptr;
  };

  ///@brief process is the main function of the callbacks and will be called
  /// with a certain type of groundplane
  /// depending on the node inputs(callbacks)
  void process(const Cloud::ConstPtr& cloud_in,
               const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
               const CameraInfo::ConstPtr& camInfo,
               const sensor_msgs::Image::ConstPtr& img = nullptr);
  /*
   * Extracts all newly arrived frames with feature points from incoming
   * tracklets The chosen features have not been observed in prior frames
   *
   * @param tracklets_in Incoming tracklets from feature matcher
   * @param newFrames a list of frames containing new feature points
   */
  std::pair<int, int> ExractNewTrackletFrames(
      const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
      std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames);

  /*
   * Estimates the depth of a list of 2d features
   *
   * @param cloud_in Current pointcloud
   * @param newFrames New Frames containing 2d-Features (contains 1 or 2 feature
   * points)
   * @param featureCount Count of new feature points
   * @param Calcuated feature depths (dimension equals the featureCount-value)
   */
  void CalculateFeatureDepthsLastFrame(
      const Cloud::ConstPtr& cloud_in_last,
      const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
      const int frameCount, Eigen::VectorXd& depths,
      Mono_Lidar::GroundPlane::Ptr& ransacPlaneOld);

  void CalculateFeatureDepthsCurFrame(
      const Cloud::ConstPtr& cloud_in_cur,
      const std::vector<std::shared_ptr<TempTrackletFrame>>& lastFrames,
      const int featureCount, Eigen::VectorXd& depths,
      Mono_Lidar::GroundPlane::Ptr& ransacPlane);

  /*
   * Converts all tracklets with a given key to the message format
   *
   * @param trackletIds Dictionary keys (Ids) of all Tracklets to convert into
   * the msg format
   * @param matchesMsg Converted Tracklets
   *
   * @return First param: Count of successfully estimated feature depths; Second
   * param: Count of features without estimated depth
   */
  std::pair<int, int> convert_tracklets_to_matches_msg(
      const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
      const std::vector<TypeTrackletKey>& trackletIds,
      matches_msg_depth_ros::MatchesMsg& matchesMsg);

  bool InitDepthEstimatorPre();
  bool InitDepthEstimatorPost();

  /*
   * Tidy up all old tracklets which have not been updated in the current frame
   *
   * @param updatedIds Ids of the tracklets which have been updated in the
   * current frame
   */
  void TidyUpTracklets(const std::vector<TypeTrackletKey>& updatedIds);

  /*
   * Adjusts the length of the timestamps to the length of the longest tracklet
   */
  void TidyUpTimeStamps();

  void SetCameraLidarTransform(const Eigen::Affine3d& camera_T_lidar);

  const Mono_Lidar::DepthCalculationStatistics& getDepthCalcStats() {
    return _depthEstimator.getDepthCalcStats();
  };

  void getCloudCameraCs(Cloud::Ptr& pointCloud_cam_cs) {
    _depthEstimator.getCloudCameraCs(pointCloud_cam_cs);
  }

  void getCloudInterpolated(Cloud::Ptr& pointCloud_interpolated) {
    _depthEstimator.getCloudInterpolated(pointCloud_interpolated);
  }

  void getPointsCloudImageCs(Eigen::Matrix2Xd& visiblePointsImageCs) {
    _depthEstimator.getPointsCloudImageCs(visiblePointsImageCs);
  }

  /*
   * Integrates the newly arrived and depth enhanced feature points into the
   * Tracklet-Dictionary
   *
   * @param newFrames Allm new frames/features
   * @param depths Calculated depths for the new features
   * @param updatedIds List of tracklet keys which have been updated in the
   * current frame
   */
  std::pair<int, int> SaveFeatureDepths(
      const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
      const Eigen::VectorXd& depthsLastFrame,
      const Eigen::VectorXd& depthsNewFrame,
      std::vector<TypeTrackletKey>& updatedIds);

 private:
  ///@brief Parameters
  tracklets_depth::TrackletDepthParameters params_;
  Mono_Lidar::DepthEstimatorParameters depth_estimator_parameters_;
  ///@brief saved ids in a map
  std::map<TypeTrackletKey, feature_tracking::Tracklet> _trackletMap;

  ///@brief Misc
  Cloud::ConstPtr _cloud_last_frame;
  Mono_Lidar::GroundPlane::Ptr groundPlaneLast_;

  std::deque<StampType> _timestamps;
  ///@brief pinhole Camera model
  std::shared_ptr<CameraPinhole> _camera{nullptr};

  ///@brief Depth Estimator object to estimate the depth of 2d feature points
  Mono_Lidar::DepthEstimator _depthEstimator;

  bool _isDepthEstimatorInitialized{false};

  ///@brief Transformation of the Lidar in the camera coordinate frame
  Eigen::Affine3d _camLidarTransform;
};

}  // namespace tracklet_depth