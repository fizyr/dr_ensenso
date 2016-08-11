#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ensenso/nxLib.h>

#include <string>

namespace dr {

/// Convert an NxLibItem to a cv::Mat.
/**
 * \throw NxError on failure.
 */
pcl::PointCloud<pcl::PointXYZ> toPointCloud(NxLibItem const & item, std::string const & what = "");

}
