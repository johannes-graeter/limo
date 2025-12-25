# Feature Tracking

This is a wrapper for feature tracking in consecutive frames. Right now, it allows LibViso mono and stereo matches to be tracked.

## Usage

* Clone libviso2 with mask interface to your catkin workspace: https://github.com/KIT-MRT/viso2.git
* Usage is obvious: check out, take header, read comments. Use.
* pushBack images to Buffer, match them, retrieve tracklets
* You can specify a mask cv::Mat(rows,cols,CV_8UC1) for defining regions in which matching shall  not be done. All maxima that have a mask value of 0 are be deleted.
* For usage see viso_feature_tracking_ros_tool

## Credits

Eike Rehder

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
