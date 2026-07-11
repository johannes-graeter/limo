# Monolidar_fusion

Calculates the depth of a feature point set from an image by using synchronized 3d-Pointclouds.

## Usage

Initialization:

1.)
call the method: "bool InitConfig(const std::string& filePath)"

This method sets all parameters of the process by using a yaml config file. 
Use "parameters.yaml" as a default parameter file.

2.) 
call the method: "bool Depthestimator::Initialize(const std::shared_ptr<CameraModel>& camera, const Eigen::Affine3d& transform_lidar_to_cam)"
@param camera: a shared pointer to a "CameraModel" object from the package "camera_models" defined in "camera_model.h"
@param transform_lidar_to_cam: an affine transformation which describes the camera pose in lidar coordinates


Usuage:

To calculate the depths of given 2d image points call:

DepthEstimator::CalculateDepth(const Cloud::ConstPtr& pointCloud, const Eigen::Matrix2Xd& points_image_cs,
			Eigen::VectorXd& points_depths, Eigen::VectorXi& resultType);

The pointcloud and the image must be synchronized in time
@param pointCloud [in] 3D pointcloud. Only points inside the camera view cone will be considered
@param points_image_cs [in] Points in image coordinates for which the depth will be calculated. Stored in row order
@param points_depths [out] Depths of the given points in meters

The method can be called once per frame in real time.

## Credits

* Alexander Wilczynski
* Johannes Gräter

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
