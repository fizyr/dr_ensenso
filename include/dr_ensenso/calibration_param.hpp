#pragma once

#include <dr_param/decompose_macros.hpp>
#include <dr_param/yaml_decompose.hpp>
#include <dr_param/yaml_preprocess.hpp>
#include <dr_param/yaml_macros.hpp>
#include <dr_eigen/yaml.hpp>

class CalibrationConfig {
public:
	bool camera_moving;
	std::string moving_frame;
	std::string fixed_frame;
	Eigen::Isometry3d camera_guess;
	Eigen::Isometry3d pattern_guess;
	std::string dump_dir;
};

class EnsensoCalibrationConfig {
public:
	CalibrationConfig initialize_config;
	bool store_calibration;
};

DR_PARAM_DEFINE_STRUCT_DECOMPOSITION(EnsensoCalibrationConfig,
(initialize_config,     "CalibrationConfig",    "", true)
(store_calibration,     "bool",                 "", false)
);

DR_PARAM_DEFINE_STRUCT_DECOMPOSITION(CalibrationConfig,
(camera_moving, "bool"  , "", true)
(moving_frame , "string", "", true)
(fixed_frame  , "string", "", true)
(camera_guess , "pose"  , "", true)
(pattern_guess, "pose"  , "", true)
(dump_dir     , "string", "", true)
);

