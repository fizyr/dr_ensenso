#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ensenso/nxLib.h>
#include <opencv2/core/core.hpp>

#include <string>

namespace dr {

/// Convert an NxLibItem to a cv::Mat.
/**
 * \throw NxError on failure.
 */
pcl::PointCloud<pcl::PointXYZ> toPointCloud(NxLibItem const & item, cv::Rect roi, std::string const & what = ""); // TODO: Remove (inc. PCL dependency) when not used anymore.

/// Load the point cloud from the camera into a buffer.
/**
 * The point cloud must have been computed before it can be loaded.
 */
void pointCloudToBuffer(
		NxLibItem const & item,
		std::string const & what,
		float* buf,         /// The buffer to load the pointcloud into.
		std::size_t width,  /// The width of the pointcloud to load.
		std::size_t height  /// The height of the pointcloud to load.
);
}
