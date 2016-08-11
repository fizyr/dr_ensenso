#include "dump.hpp"
#include "opencv.hpp"
#include "pcl.hpp"
#include "util.hpp"

#include <pcl/io/vtk_lib_io.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/time_facet.hpp>

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <sstream>
#include <iostream>

namespace dr {

namespace {
	std::string formatTime(boost::posix_time::ptime timestamp, std::string const & format) {
		std::stringstream buffer;
		buffer.imbue(std::locale(buffer.getloc(), new boost::posix_time::time_facet(format.c_str())));
		buffer << timestamp;
		return buffer.str();
	}

	std::string formatTime(std::int64_t timestamp, std::string const & format) {
		return formatTime(boost::posix_time::ptime(boost::gregorian::date(1970, 1, 1), boost::posix_time::microseconds(timestamp)), format);
	}
}

void dumpCloud(NxLibItem const & item, std::string const & path_prefix, std::string const & path_suffix, std::string const & time_format) {
	pcl::PointCloud<pcl::PointXYZ> cloud = toPointCloud(item);
	std::string filename = path_prefix + formatTime(cloud.header.stamp, time_format) + path_suffix + ".pcd";
	boost::filesystem::create_directories(boost::filesystem::path(filename).parent_path());
	pcl::io::savePCDFileASCII(filename, cloud);
}

void dumpImage(NxLibItem const & item, std::string const & path_prefix, std::string const & path_suffix, std::string const & time_format) {
	std::int64_t timestamp = getNxBinaryTimestamp(item);
	std::string filename = path_prefix + formatTime(timestamp, time_format) + path_suffix + ".png";
	cv::Mat image = toCvMat(item);
	cv::imwrite(filename, image);
}

void dumpStereoImages(NxLibItem const & item, std::string const & path_prefix, std::string const & path_suffix, std::string const & time_format) {
	NxLibCommand command(cmdSaveImage);
	if (item.isObject()) {
		dumpImage(item[itmLeft],  path_prefix, path_suffix + "_left",  time_format);
		dumpImage(item[itmRight], path_prefix, path_suffix + "_right", time_format);
	} else {
		for (int i = 0; i < item.count(); ++i) {
			dumpImage(item[i][itmLeft],  path_prefix, path_suffix + "_left_"  + std::to_string(i), time_format);
			dumpImage(item[i][itmRight], path_prefix, path_suffix + "_right_" + std::to_string(i), time_format);
		}
	}
}

void dumpCameraImages(
	NxLibItem const & item,
	std::string const & path_prefix,
	std::string const & path_suffix,
	std::string const & time_format,
	bool raw,
	bool rectified,
	bool disparity,
	bool point_cloud
) {
	std::string type = getNx<std::string>(item[itmType]);
	if (type == valStereo) {
		std::string serial = getNx<std::string>(item[itmSerialNumber]);
		if (raw)         dumpStereoImages(item[itmImages][itmRaw],       path_prefix, path_suffix + "_" + serial + "_raw",         time_format);
		if (rectified)   dumpStereoImages(item[itmImages][itmRectified], path_prefix, path_suffix + "_" + serial + "_rectified",   time_format);
		if (point_cloud) dumpCloud(item[itmImages][itmPointMap],         path_prefix, path_suffix + "_" + serial + "_point_cloud", time_format);
		if (disparity)   dumpImage(item[itmImages][itmDisparityMap],     path_prefix, path_suffix + "_" + serial + "_disparity",   time_format);

	} else if (type == valMonocular) {
		std::string serial = getNx<std::string>(item[itmSerialNumber]);
		if (raw)       dumpImage(item[itmImages][itmRaw],       path_prefix, path_suffix + "_" + serial + "_raw",       time_format);
		if (rectified) dumpImage(item[itmImages][itmRectified], path_prefix, path_suffix + "_" + serial + "_rectified", time_format);

	} else {
		throw std::runtime_error("Unknown camera type: " + type + ". Can not dump camera images.");
	}
}

void dumpParameters(
	NxLibItem const & item,
	std::string const & path_prefix,
	std::string const & path_suffix,
	std::string const & time_format
) {
	std::string filename = path_prefix + formatTime(boost::posix_time::microsec_clock::universal_time(), time_format) + path_suffix + ".json";
	writeNxJsonToFile(item, filename);
}

void dumpParametersYaml(
	NxLibItem const & item,
	std::string const & camera,
	std::string const & path_prefix,
	std::string const & path_suffix,
	std::string const & time_format
) {
	std::string filename = path_prefix + formatTime(boost::posix_time::microsec_clock::universal_time(), time_format) + path_suffix + "_" + camera + ".yaml";

	cv::FileStorage file(filename, cv::FileStorage::WRITE);
	if (!file.isOpened()) {
		throw std::runtime_error("Failed to open file " + filename);
	}

	file << "camera_matrix" << toCameraMatrix(item, camera);
	file << "distortion_coefficients" << toDistortionParameters(item, camera);
	file << "rectification_matrix" << toRectificationMatrix(item, camera);
	file << "projection_matrix" << toProjectionMatrix(item, camera);

	file.release();
}

}
