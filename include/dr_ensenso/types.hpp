#pragma once

#include "ensenso.hpp"

#include <estd/result.hpp>

namespace dr {

using Error = estd::error;

template<typename T>
using Result = estd::result<T, Error>;

using ImageType = dr::ImageType;

inline ImageType parseImageType(std::string const & name) {
	if (name == "stereo_raw_left"       ) return ImageType::stereo_raw_left;
	if (name == "stereo_raw_right"      ) return ImageType::stereo_raw_right;
	if (name == "stereo_rectified_left" ) return ImageType::stereo_rectified_left;
	if (name == "stereo_rectified_right") return ImageType::stereo_rectified_right;
	if (name == "disparity"             ) return ImageType::disparity;
	if (name == "monocular_raw"         ) return ImageType::monocular_raw;
	if (name == "monocular_rectified"   ) return ImageType::monocular_rectified;
	throw std::runtime_error("Unknown image type: " + name);
}

inline bool isMonocular(ImageType type) {
	switch (type) {
		case ImageType::stereo_raw_left:
		case ImageType::stereo_raw_right:
		case ImageType::stereo_rectified_left:
		case ImageType::stereo_rectified_right:
		case ImageType::disparity:
			return false;
		case ImageType::monocular_raw:
		case ImageType::monocular_rectified:
		case ImageType::monocular_overlay:
			return true;
	}

	throw std::runtime_error("Unknown image type: " + std::to_string(int(type)));
}

inline bool needsRectification(ImageType type) {
	switch (type) {
		case ImageType::stereo_rectified_left:
		case ImageType::stereo_rectified_right:
		case ImageType::disparity:
		case ImageType::monocular_rectified:
		case ImageType::monocular_overlay:
			return true;
		case ImageType::monocular_raw:
		case ImageType::stereo_raw_left:
		case ImageType::stereo_raw_right:
			return false;
	}
	throw std::runtime_error("Unknown image type: " + std::to_string(int(type)));
}

}
