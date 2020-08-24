#pragma once

#include <estd/result.hpp>

namespace dr {

using Error = estd::error;

template<typename T>
using Result = estd::result<T, Error>;

enum class ImageType {
	stereo_raw_left,
	stereo_raw_right,
	stereo_rectified_left,
	stereo_rectified_right,
	disparity,
	monocular_raw,
	monocular_rectified,
	monocular_overlay,
};

inline Result<ImageType> parseImageType(std::string const & name) {
	if (name == "stereo_raw_left"       ) return ImageType::stereo_raw_left;
	if (name == "stereo_raw_right"      ) return ImageType::stereo_raw_right;
	if (name == "stereo_rectified_left" ) return ImageType::stereo_rectified_left;
	if (name == "stereo_rectified_right") return ImageType::stereo_rectified_right;
	if (name == "disparity"             ) return ImageType::disparity;
	if (name == "monocular_raw"         ) return ImageType::monocular_raw;
	if (name == "monocular_rectified"   ) return ImageType::monocular_rectified;
	return estd::error("unknown image type: " + name);
}

inline Result<bool> isMonocular(ImageType type) {
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
	return estd::error("unknown image type: " + std::to_string(int(type)));
}

inline Result<bool> needsRectification(ImageType type) {
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
	return estd::error("unknown image type: " + std::to_string(int(type)));
}

}
