#pragma once

#include <ensenso/nxLib.h>
#include <opencv2/core/core.hpp>

namespace dr {

/// Convert an NxLibItem to a cv::Mat.
/**
 * \throw NxError on failure.
 */
cv::Mat toCvMat(NxLibItem const & item, std::string const & what = "");

/// Convert a NxLibItem containing camera matrix to a cv::Mat.
/**
 * The camera matrix corresponds to the K parameter in OpenCV.
 * \throw NxError on failure.
 */
cv::Mat toCameraMatrix(NxLibItem const & item, std::string const & camera = "Left", std::string const & what = "");

/// Convert a NxLibItem containing the distortion parameters to a cv::Mat.
/**
 * The distortion parameters correspond to the D parameter in OpenCV.
 * \throw NxError on failure.
 */
cv::Mat toDistortionParameters(NxLibItem const & item, std::string const & camera = "Left", std::string const & what = "");

/// Convert a NxLibItem containing the projection matrix to a cv::Mat.
/**
 * The projection matrix corresponds to the P matrix in OpenCV.
 * \throw NxError on failure.
 */
cv::Mat toProjectionMatrix(NxLibItem const & item, std::string const & camera = "Left", std::string const & what = "");

/// Convert a NxLibItem containing the rectification matrix to a cv::Mat.
/**
 * The rectification matrix corresponds to the R matrix in OpenCV.
 * \throw NxError on failure.
 */
cv::Mat toRectificationMatrix(NxLibItem const & item, std::string const & camera = "Left", std::string const & what = "");


}
