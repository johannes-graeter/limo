#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include "tracklets_depth/tracklet_depth_module.h"

namespace tracklets_depth {
namespace {
CameraPinhole::Ptr CreateCameraPinholeFromCameraInfo(const sensor_msgs::CameraInfo::ConstPtr& cam_info) {
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(cam_info);
    assert(model.fx() == model.fy()); // we only support undistorted images

    // Extract camera parameters from calibration matrix
    double focalLengthX = model.fx();
    double principlePointX = model.cx();
    double principlePointY = model.cy();

    return std::make_shared<CameraPinhole>(
        cam_info->width, cam_info->height, focalLengthX, principlePointX, principlePointY);
}
}

std::pair<int, int> TrackletDepthModule::ExractNewTrackletFrames(
    const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
    std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames) {
  int frameCountNew = 0, frameCountOld = 0;

  for (const auto& tracklet : tracklets_in->tracks) {
    int id = tracklet.id;

    // check if tracklet already exists in cache
    bool trackExists = _trackletMap.count(id);

    std::shared_ptr<TempTrackletFrame> tempFrame;

    if (!trackExists) {
      // add the last two features if tracklet is completely new
      const auto& matchNew = tracklet.feature_points.at(0);
      const auto& matchOld = tracklet.feature_points.at(1);

      tempFrame = std::make_unique<TempTrackletFrameExp>(
          id, std::make_pair(matchNew.u, matchNew.v),
          std::make_pair(matchOld.u, matchOld.v));

      frameCountNew++;
      frameCountOld++;
    } else {
      // add the newest feature if trcaklet already exists
      const auto& matchNew = tracklet.feature_points.at(0);

      tempFrame = std::make_unique<TempTrackletFrame>(
          id, std::make_pair(matchNew.u, matchNew.v));

      frameCountNew++;
    }

    newFrames.push_back(tempFrame);
  }

  return std::make_pair(frameCountNew, frameCountOld);
}

void TrackletDepthModule::CalculateFeatureDepthsCurFrame(
    const Cloud::ConstPtr& cloud_in_cur,
    const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
    const int frameCount, Eigen::VectorXd& depths,
    Mono_Lidar::GroundPlane::Ptr& ransacPlane) {
  // Convert the feature points to the interface format for the DepthEstimator
  depths.resize(frameCount);
  Eigen::Matrix2Xd featureCoordinates(2, frameCount);

  int i = 0;
  for (const auto& featureNew : newFrames) {
    // insert features of the current frame
    featureCoordinates(0, i) = featureNew->_feature.first;
    featureCoordinates(1, i) = featureNew->_feature.second;
    i++;
  }

  this->_depthEstimator.CalculateDepth(cloud_in_cur, featureCoordinates, depths,
                                       ransacPlane);
}

void TrackletDepthModule::CalculateFeatureDepthsLastFrame(
    const Cloud::ConstPtr& cloud_in_last,
    const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
    const int frameCount, Eigen::VectorXd& depths,
    Mono_Lidar::GroundPlane::Ptr& ransacPlaneOld) {
  // Convert the feature points to the interface format for the DepthEstimator
  depths.resize(frameCount);

  if (cloud_in_last == nullptr) {
    depths.setConstant(-1);
    return;
  }

  Eigen::Matrix2Xd featureCoordinates(2, frameCount);

  int i = 0;
  for (const auto& featureNew : newFrames) {
    // if the tracklet of the frame has been added this frame, there are exist
    // two new features in successive frames
    const auto featureLast =
        std::dynamic_pointer_cast<TempTrackletFrameExp>(featureNew);

    if (featureLast != nullptr) {
      // feature from a new tracklet
      // Add feature of the last frame
      featureCoordinates(0, i) = featureLast->_featureLast.first;
      featureCoordinates(1, i) = featureLast->_featureLast.second;
      i++;
    }
  }

  this->_depthEstimator.CalculateDepth(cloud_in_last, featureCoordinates,
                                       depths, ransacPlaneOld);
}

std::pair<int, int> TrackletDepthModule::SaveFeatureDepths(
    const std::vector<std::shared_ptr<TempTrackletFrame>>& newFrames,
    const Eigen::VectorXd& depthsLastFrame,
    const Eigen::VectorXd& depthsCurFrame,
    std::vector<TypeTrackletKey>& updatedIds) {
  int i = 0, j = 0;
  int newCount = 0;
  int oldCount = 0;

  for (const auto featureNew : newFrames) {
    int id = featureNew->_keyFrameId;

    const auto featureLast =
        std::dynamic_pointer_cast<TempTrackletFrameExp>(featureNew);

    if (featureLast != nullptr) {
      // New tracklet --> Create
      _trackletMap.emplace(id, feature_tracking::Tracklet());
      _trackletMap[id].id_ = id;
      _trackletMap[id].age_ = 0;

      int u = featureLast->_featureLast.first;
      int v = featureLast->_featureLast.second;

      auto match = feature_tracking::Match(u, v);
      // use world point to save feature's depth
      match.x_ = std::make_shared<feature_tracking::WorldPoint>();
      match.x_->data[2] = depthsLastFrame[j];
      _trackletMap[id].push_front(match);

      j++;
      newCount++;
    } else
      oldCount++;

    int u = featureNew->_feature.first;
    int v = featureNew->_feature.second;
    auto match = feature_tracking::Match(u, v);
    // use world point to save feature's depth temporarily
    match.x_ = std::make_shared<feature_tracking::WorldPoint>();
    match.x_->data[2] = depthsCurFrame[i];
    _trackletMap[id].push_front(match);

    i++;

    // mark tracklet to send for this frame
    updatedIds.push_back(id);
  }

  return std::make_pair(oldCount, newCount);
}

void TrackletDepthModule::TidyUpTracklets(
    const std::vector<TypeTrackletKey>& updatedIds) {
  std::vector<TypeTrackletKey> toDelete;

  for (const auto& keyAvailable : _trackletMap) {
    toDelete.push_back(keyAvailable.first);
  }

  for (const auto& keyStay : updatedIds) {
    for (int i = 0; i < int(toDelete.size()); i++) {
      if (toDelete.at(i) == keyStay) {
        toDelete.erase(toDelete.begin() + i);
        break;
      }
    }
  }

  // delete old tracklets which haven't been updated this frame
  for (const auto id : toDelete) {
    auto it = _trackletMap.find(id);
    _trackletMap.erase(it);
  }
}

void TrackletDepthModule::TidyUpTimeStamps() {
  int maxLength = 0;

  for (const auto& tracklet : _trackletMap) {
    if (int(tracklet.second.size()) > maxLength) {
      maxLength = tracklet.second.size();
    }
  }

  while (int(_timestamps.size()) > maxLength) {
    _timestamps.pop_back();
  }
}

std::pair<int, int> TrackletDepthModule::convert_tracklets_to_matches_msg(
    const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
    const std::vector<TypeTrackletKey>& trackletIds,
    matches_msg_depth_ros::MatchesMsg& Out) {
  using MsgFeature = matches_msg_depth_ros::FeaturePoint;
  using MsgTracklet = matches_msg_depth_ros::Tracklet;

  Out.tracks.reserve(trackletIds.size());

  int success = 0;
  int failed = 0;

  // Insert tracklets to new message
  for (const auto trackletId : trackletIds) {
    const auto& curTracklet = _trackletMap[trackletId];
    MsgTracklet t;

    t.id = curTracklet.id_;
    t.age = curTracklet.age_;
    t.feature_points.reserve(curTracklet.size());

    for (const auto& cur_match : curTracklet) {
      const auto& cur_point = cur_match.p1_;
      const float depth = cur_match.x_->data[2];
      MsgFeature msg_feature;

      depth >= 0 ? success++ : failed++;

      msg_feature.u = float(cur_point.u_);
      msg_feature.v = float(cur_point.v_);
      msg_feature.d = depth;

      t.feature_points.push_back(std::move(msg_feature));
    }

    assert(t.feature_points.size() == curTracklet.size());
    Out.tracks.push_back(std::move(t));
  }

  assert(Out.tracks.size() == trackletIds.size());

  // Insert timestamps
  for (const auto& stamp : tracklets_in->stamps) {
    Out.stamps.push_back(stamp);
  }

  // Init header of new message
  Out.header = tracklets_in->header;

  return std::make_pair(success, failed);
}

void TrackletDepthModule::process(
    const Cloud::ConstPtr& cloud_in,
    const matches_msg_ros::MatchesMsgConstPtr& tracklets_in,
    const CameraInfo::ConstPtr& camInfo,
    const sensor_msgs::Image::ConstPtr& img) {
  if (cloud_in->points.size() < 100) {
    ROS_WARN_STREAM("In tracklet_depth: received less than 100 points.");
  }
  Mono_Lidar::GroundPlane::Ptr gp;
  // Init groundplane.
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camInfo);
  Mono_Lidar::SemanticPlane::Camera cam;
  cam.f = model.fx();
  cam.cu = model.cx();
  cam.cv = model.cy();
  cam.transform_cam_lidar = _camLidarTransform;
  cv_bridge::CvImageConstPtr img_ptr =
      cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
  std::set<int> gp_labels{6, 7, 8, 9};
  double plane_inlier_threshold =
      depth_estimator_parameters_.ransac_plane_refinement_treshold;
  gp = std::make_shared<Mono_Lidar::SemanticPlane>(
      img_ptr->image, cam, gp_labels, plane_inlier_threshold);

  // Timer
  auto start_time = std::chrono::steady_clock::now();

  std::stringstream stats;

  _timestamps.push_front(tracklets_in->header.stamp);

  // Initialize depthestimator it not done
  if (!_isDepthEstimatorInitialized) {
    if (_camera==nullptr){
      _camera = CreateCameraPinholeFromCameraInfo(camInfo);
    }
    InitDepthEstimatorPost();
    _isDepthEstimatorInitialized=true;
  }

  double sec = tracklets_in->header.stamp.toSec();
  stats << "Received tracklet-lidar pair at: " << sec << std::endl;
  stats << "Tracklet length: " << tracklets_in->tracks.size() << std::endl;

  // Save all new feature points of the incoming tracklets which haven't been
  // estimated in previous frames
  std::vector<std::shared_ptr<tracklets_depth::TempTrackletFrame>> tempFrames;
  auto frameCount = ExractNewTrackletFrames(tracklets_in, tempFrames);
  int frameCountNew = frameCount.first;
  int frameCountOld = frameCount.second;

  Eigen::VectorXd depthsCurFrame, depthsLastFrame;

  try {
    // Use the DepthEstimator to calculate the depth all newly arrived feature
    // points
    CalculateFeatureDepthsLastFrame(_cloud_last_frame, tempFrames,
                                    frameCountOld, depthsLastFrame,
                                    groundPlaneLast_);
  } catch (const Mono_Lidar::GroundPlane::ExceptionPclInvalid& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_WARN_STREAM(
        "TrackletDepthInterface: Old frame continue with invalid depths");

    depthsLastFrame.resize(frameCountOld);
    depthsLastFrame.setConstant(-1);
  }
  try {
    CalculateFeatureDepthsCurFrame(cloud_in, tempFrames, frameCountNew,
                                   depthsCurFrame, gp);
    if (gp == nullptr) {
      ROS_ERROR_STREAM("TrackletDepthInterface: Plane not calculated");
    }

    // remember cloud
    _cloud_last_frame = cloud_in;
  } catch (const Mono_Lidar::GroundPlane::ExceptionPclInvalid& e) {
    ROS_ERROR_STREAM(e.what());
    ROS_WARN_STREAM(
        "TrackletDepthInterface: Cur frame continue with invalid depths");

    depthsCurFrame.resize(frameCountNew);
    depthsCurFrame.setConstant(-1);

    // mark plane invalid so it will be recalculated next time.
    gp = nullptr;

    // declare cloud invalid
    _cloud_last_frame = nullptr;
  }

  // remember the ransac plane
  groundPlaneLast_ = gp;

  // Write Results
  std::vector<TypeTrackletKey> updatetIds;
  auto trackletsCount = SaveFeatureDepths(tempFrames, depthsLastFrame,
                                          depthsCurFrame, updatetIds);

  stats << "Old tracklets Count: " << trackletsCount.first << std::endl;
  stats << "New tracklets Count: " << trackletsCount.second << std::endl;

  // Convert newly updated tracklets to the msg format and publish
  matches_msg_depth_ros::MatchesMsg msgOut;
  auto matchesSuccess =
      convert_tracklets_to_matches_msg(tracklets_in, updatetIds, msgOut);

  stats << "Feature Estimation success count: " << matchesSuccess.first
        << std::endl;
  stats << "Feature Estimation fail count: " << matchesSuccess.second
        << std::endl;

  // Delete old tracklets (tracklets with no updates in the current frame)
  TidyUpTracklets(updatetIds);
  TidyUpTimeStamps();

  //    if (groundPlaneLast_ != nullptr) {
  //        std::stringstream ss;
  //        ss << "/tmp/gp.txt";
  //        std::ofstream file(ss.str().c_str(), std::ios_base::app);
  //        file.precision(12);
  //        Eigen::Vector4f plane_params = groundPlaneLast_->getModelCoeffs();
  //        file << plane_params[0] << " " << plane_params[1] << " " <<
  //        plane_params[2] << " " << plane_params[3]
  //             << std::endl;
  //        file.close();
  //    }

  ROS_DEBUG_STREAM("TrackletDepthInterfaceRosTool: " + stats.str());
  ROS_INFO_STREAM("Duration tracklet_depth_ros_tool="
                  << std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - start_time)
                         .count()
                  << " ms");
}

bool TrackletDepthModule::InitDepthEstimatorPost() {
  std::cout << "Initialize DepthEstimator Post" << std::endl;

  if (!_depthEstimator.Initialize(_camera, _camLidarTransform))
    throw "Error in 'Initialize' of DepthEstimator.";

  std::cout << "DepthEstimator successful initialized" << std::endl;

  _isDepthEstimatorInitialized = true;

  return true;
}

bool TrackletDepthModule::InitDepthEstimatorPre() {
  std::cout << "Initialize DepthEstimator Pre" << std::endl;
  if (!_depthEstimator.InitConfig(std::make_shared<Mono_Lidar::DepthEstimatorParameters>(depth_estimator_parameters_))){
    throw "Error in 'initConfig' of DepthEstimator.";
  }

  return true;
}

void TrackletDepthModule::SetCameraLidarTransform(const Eigen::Affine3d& camera_T_lidar) {
      _camLidarTransform = camera_T_lidar;
}
}
