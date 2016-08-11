#pragma once
#include <string>

#include <ensenso/nxLib.h>
namespace dr {

	char const * default_time_format = "%Y-%m-%d %H-%M-%S.%f";

	/// Dump a point cloud with a given description.
	void dumpCloud(
		NxLibItem const & item,                                ///< An NxLibItem holding a point map.
		std::string const & path_prefix,                       ///< A string to prepend before the timestamp to form the filename.
		std::string const & path_suffix,                       ///< A string to append after the timestamp to form the filename.
		std::string const & time_format = default_time_format  ///< A Boost DateTime compatible format string for time formatting.
	);

	/// Dump an image from a binary node with a given description.
	void dumpImage(
		NxLibItem const & item,                                ///< An NxLibItem holding a image.
		std::string const & path_prefix,                       ///< A string to prepend before the timestamp to form the filename.
		std::string const & path_suffix,                       ///< A string to append after the timestamp to form the filename.
		std::string const & time_format = default_time_format  ///< A Boost DateTime compatible format string for time formatting.
	);

	/// Dump all stereo images from a node, possibly once for each FlewView image.
	void dumpStereoImages(
		NxLibItem const & item,                                ///< An NxLibItem holding a stereo images.
		std::string const & path_prefix,                       ///< A string to prepend before the timestamp to form the filename.
		std::string const & path_suffix,                       ///< A string to append after the timestamp to form the filename.
		std::string const & time_format = default_time_format  ///< A Boost DateTime compatible format string for time formatting.
	);

	/// Dump all camera images.
	void dumpCameraImages(
		NxLibItem const & item,                                ///< An NxLibItem representing a camera.
		std::string const & path_prefix,                       ///< A string to prepend before the timestamp to form the filename.
		std::string const & path_suffix,                       ///< A string to append after the timestamp to form the filename.
		std::string const & time_format = default_time_format, ///< A Boost DateTime compatible format string for time formatting.
		bool raw         = true,                               ///< If true, dump the raw image(s).
		bool rectified   = true,                               ///< If true, dump the rectified image(s).
		bool disparity   = true,                               ///< If true, dump the disparity image (only for stereo cameras).
		bool point_cloud = true                                ///< If true, dump the point cloud (only for stereo cameras).
	);

	/// Dump the parameters of a camera.
	void dumpParameters(
		NxLibItem const & item,                                ///< An NxLibItem representing the camera parameters.
		std::string const & path_prefix,                       ///< A string to prepend before the timestamp to form the filename.
		std::string const & path_suffix,                       ///< A string to append after the timestamp to form the filename.
		std::string const & time_format = default_time_format  ///< A Boost DateTime compatible format string for time formatting.
	);

	/// Dump the parameters of a camera as a OpenCV yaml.
	void dumpParametersYaml(
		NxLibItem const & item,                                ///< An NxLibItem representing the camera parameters.
		std::string const & camera,                            ///< Camera for which to dump parameters.
		std::string const & path_prefix,                       ///< A string to prepend before the timestamp to form the filename.
		std::string const & path_suffix,                       ///< A string to append after the timestamp to form the filename.
		std::string const & time_format = default_time_format  ///< A Boost DateTime compatible format string for time formatting.
	);

}
