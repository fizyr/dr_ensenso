// Must include opencv version information before nxLib, so make it the first include.
#include <opencv2/opencv.hpp>
#include "opencv.hpp"
#include "util.hpp"

namespace dr {

Result<cv::Mat> toCvMat(NxLibItem const & item, std::optional<cv::Rect> roi) {
	int error = 0;
	cv::Mat result;
	item.getBinaryData(&error, result, nullptr);
	if (error) return estd::error(composeTreeReadErrorMessage(error, item));

	// convert RGB output from camera to OpenCV standard (BGR)
	if (result.channels() == 3) {
		cv::cvtColor(result, result, cv::COLOR_RGB2BGR);
	}

	if (roi) {
		if (!roi->empty() && ((roi->width) != result.size().width || (roi->height) != result.size().height)) {
			result = result(*roi);
		}
	}

	return result;
}

Result<void> toCvMat(
	NxLibItem const & item,
	std::uint8_t * pointer,
	std::size_t width,
	std::size_t height,
	int cv_type,
	std::optional<cv::Rect> roi
) {
	int error = 0;

	cv::Mat wrapped(height, width, cv_type, pointer);
	if (roi && !roi->empty()) {
		if (height != std::size_t(roi->height) || width != std::size_t(roi->width)) {
			return estd::error(
				"ROI height: " + std::to_string(roi->height)
				+ " and width: " + std::to_string(roi->width)
				+ " differs from input height: " + std::to_string(height)
				+ " and width: " + std::to_string(width)
			);
		}

		// Retrieve metadata.
		int initial_height, initial_width, channels, element_width;
		bool is_float;
		double timestamp;
		item.getBinaryDataInfo(&error, &initial_width, &initial_height, &channels, &element_width, &is_float, &timestamp);
		if (error) return estd::error(composeTreeReadErrorMessage(error, item));

		if (((roi->width) != initial_width || (roi->height) != initial_height)) {
			cv::Mat original(initial_height, initial_width, cv_type);
			item.getBinaryData(&error, original, nullptr);
			if (error) return estd::error(composeTreeReadErrorMessage(error, item));

			original(*roi).copyTo(wrapped);
			if (wrapped.channels() == 3) cv::cvtColor(wrapped, wrapped, cv::COLOR_RGB2BGR);
			return estd::in_place_valid;
		}
	}

	item.getBinaryData(&error, wrapped, nullptr);
	if (error) return estd::error(composeTreeReadErrorMessage(error, item));

	// convert RGB output from camera to OpenCV standard (BGR)
	if (wrapped.channels() == 3) cv::cvtColor(wrapped, wrapped, cv::COLOR_RGB2BGR);
	return estd::in_place_valid;
}

Result<cv::Mat> toCameraMatrix(NxLibItem const & item, std::string const & camera) {
	int error = 0;
	cv::Mat result = cv::Mat::zeros(3, 3, CV_64F);
	for (std::size_t i = 0; i < 3; i++) {
		for (std::size_t j = 0; j < 3; j++) {
			result.at<double>(i,j) = item[itmMonocular][camera == "Left" ? itmLeft : itmRight][itmCamera][j][i].asDouble(&error);
			if (error) return estd::error(composeTreeReadErrorMessage(error, item));
		}
	}
	return result;
}

Result<cv::Mat> toDistortionParameters(NxLibItem const & item, std::string const & camera) {
	int error = 0;
	cv::Mat result = cv::Mat::zeros(8, 1, CV_64F);
	for (std::size_t i = 0; i < 5; i++) {
		result.at<double>(i) = item[itmMonocular][camera == "Left" ? itmLeft : itmRight][itmDistortion][i].asDouble(&error);
		if (error) return estd::error(composeTreeReadErrorMessage(error, item));
	}
	return result;
}

Result<cv::Mat> toRectificationMatrix(NxLibItem const & item, std::string const & camera) {
	int error = 0;
	cv::Mat result = cv::Mat::zeros(3, 3, CV_64F);
	for (std::size_t i = 0; i < 3; i++) {
		for (std::size_t j = 0; j < 3; j++) {
			result.at<double>(i,j) = item[itmStereo][camera == "Left" ? itmLeft : itmRight][itmRotation][j][i].asDouble(&error);
			if (error) return estd::error(composeTreeReadErrorMessage(error, item));
		}
	}
	return result;
}

Result<cv::Mat> toProjectionMatrix(NxLibItem const & item, std::string const & camera) {
	int error = 0;
	cv::Mat result = cv::Mat::zeros(3, 4, CV_64F);
	for (std::size_t i = 0; i < 3; i++) {
		for (std::size_t j = 0; j < 3; j++) {
			result.at<double>(i,j) = item[itmStereo][camera == "Left" ? itmLeft : itmRight][itmCamera][j][i].asDouble(&error);
			if (error) return estd::error(composeTreeReadErrorMessage(error, item));
		}
	}

	if (camera == "Right") {
		double B = item[itmStereo][itmBaseline].asDouble() / 1000.0;
		double fx = result.at<double>(0,0);
		result.at<double>(0,3) = (-fx * B);
	}
	return result;
}

Result<void> toNxLibItem(NxLibItem const & item, cv::Mat const & value) {
	int error = 0;
	item.setBinaryData(&error, value);
	if (error) return estd::error(composeTreeWriteErrorMessage(error, item));
	return estd::in_place_valid;
}

}
