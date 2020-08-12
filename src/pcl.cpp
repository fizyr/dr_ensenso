#include "pcl.hpp"
#include "util.hpp"

#include <stdexcept>

namespace dr {

namespace {
	/// Conversion from ensenso timestamp to PCL timestamp.
	std::uint64_t ensensoStampToPcl(double stamp) { return (stamp - 11644473600.0) * 1000000.0; }
}

Result<void> pointCloudToBuffer(
		NxLibItem const & item,
		std::string const & what,
		float* buf,
		std::size_t width,
		std::size_t height,
		std::optional<cv::Rect> roi
	) {
	int error = 0;

	// TODO: Move the item functions to util.cpp ??
	// Retrieve metadata.
	int actual_height, actual_width, channels, element_width;
	bool is_float;
	double timestamp;
	item.getBinaryDataInfo(&error, &actual_width, &actual_height, &channels, &element_width, &is_float, &timestamp);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}

	// Make sure data is what we expect.
	std::string what2 = what.empty() ? std::string() : ": " + what;
	if (channels != 3) {
		return estd::error("unexpected number of channels: " + std::to_string(channels) + ", expected 3" + what2);
	}
	if (!is_float) {
		return estd::error("expected floating point data for point cloud conversion" + what2);
	}
	if (element_width != 4) {
		return estd::error("unexpected data width: " + std::to_string(element_width) + ", expected 4" + what2);
	}
	if (!roi) {
		if (std::size_t(actual_height) != height) {
			return estd::error("unexpected height: " + std::to_string(actual_height) + ", asked for height: " + std::to_string(height) + what2);
		}
		if (std::size_t(actual_width) != width) {
			return estd::error("unexpected width: "  + std::to_string(actual_width) + ", asked for width: "  + std::to_string(width) + what2);
		}
	}

	// Retrieve data.
	std::vector<float> point_list;
	item.getBinaryData(&error, point_list, 0);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}

	// Copy data in padded buffer (and convert millimeters to meters)
	if (roi) {
		for (int i = 0; i < roi->height * roi->width; ++i) {
			int point_list_index = actual_width * (roi->tl().y + floor(i / roi->width)) + roi->tl().x + (i % roi->width);
			int cur_buf_index = i * 4;
			int cur_list_index = point_list_index * 3;
			buf[cur_buf_index] = point_list[cur_list_index] / 1000.0;
			buf[cur_buf_index+1] = point_list[cur_list_index + 1] / 1000.0;
			buf[cur_buf_index+2] = point_list[cur_list_index + 2] / 1000.0;
			buf[cur_buf_index+3] = 1;
		}
	} else {
		for (std::size_t i = 0; i < point_list.size() / 3; i++) {
			int cur_buf_index = i * 4;
			int cur_list_index = i * 3;
			buf[cur_buf_index] = point_list[cur_list_index] / 1000.0;
			buf[cur_buf_index+1] = point_list[cur_list_index + 1] / 1000.0;
			buf[cur_buf_index+2] = point_list[cur_list_index + 2] / 1000.0;
			buf[cur_buf_index+3] = 1;
		}
	}
	return estd::in_place_valid;
}

Result<pcl::PointCloud<pcl::PointXYZ>> toPointCloud(NxLibItem const & item, std::optional<cv::Rect> roi, std::string const & what) {
	int error = 0;

	// Retrieve metadata.
	int height, width, channels, element_width;
	bool is_float;
	double timestamp;
	item.getBinaryDataInfo(&error, &width, &height, &channels, &element_width, &is_float, &timestamp);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}

	// Make sure data is what we expect.
	std::string what2 = what.empty() ? std::string() : ": " + what;
	if (channels != 3) {
		return estd::error("unexpected number of channels: " + std::to_string(channels) + ", expected 3" + what2);
	}
	if (!is_float) {
		return estd::error("expected floating point data for point cloud conversion" + what2);
	}
	if (element_width != 4) {
		return estd::error("unexpected data width: " + std::to_string(element_width) + ", expected 4" + what2);
	}

	// Retrieve data.
	std::vector<float> point_list;
	item.getBinaryData(&error, point_list, 0);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}

	// Copy point cloud and convert in meters
	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.header.stamp    = ensensoStampToPcl(timestamp);
	cloud.header.frame_id = "/camera_link";
	cloud.width           = width;
	cloud.height          = height;
	cloud.is_dense        = false;
	cloud.resize(height * width);

	// Copy data in point cloud (and convert milimeters in meters)
	for (size_t i = 0; i < point_list.size (); i += 3) {
		cloud.points[i / 3].x = point_list[i] / 1000.0;
		cloud.points[i / 3].y = point_list[i + 1] / 1000.0;
		cloud.points[i / 3].z = point_list[i + 2] / 1000.0;
	}

	if (!roi) return cloud;
	if (roi->empty() || ((roi->width)  == width && (roi->height) == height))  return cloud;
	else {
		pcl::PointCloud<pcl::PointXYZ> cropped_cloud;
		cropped_cloud.header.stamp    = cloud.header.stamp;
		cropped_cloud.header.frame_id = cloud.header.frame_id;
		cropped_cloud.width           = roi->width;
		cropped_cloud.height          = roi->height;
		cropped_cloud.is_dense        = false;
		cropped_cloud.resize(cropped_cloud.height * cropped_cloud.width);

		for (int i = 0; i < roi->height; ++i) {
			for (int j = 0; j < roi->width; ++j) {
				cropped_cloud.points[(i * cropped_cloud.width) + j] = cloud.points[((roi->tl().y + i) * cloud.width) + roi->tl().x + j];
			}
		}

		return cropped_cloud;
	}
}

}
