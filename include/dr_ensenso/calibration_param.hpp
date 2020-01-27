#pragma once

#include <dr_param/decompose_macros.hpp>
#include <dr_param/yaml_decompose.hpp>
#include <dr_param/yaml_preprocess.hpp>
#include <dr_param/yaml_macros.hpp>
#include <dr_eigen/yaml.hpp>

struct InitializeCalibrationConfig {
	/// If true, the camera is attached to the moving frame; if false, it is attached to the fixed frame.
	/// Additionally, the calibration plate is assumed to be attached to the other frame.
	bool camera_moving;

	/// Name of the moving frame (usually the name of the robot end effector).
	std::string moving_frame;

	/// Name of the fixed frame (usually the name of the robot base frame).
	std::string fixed_frame;

	/// Initial guess of the pose of the camera, can be used to speed up the calibration optimization.
	/// Depending on the value of camera_moving, this pose is defined in either moving_frame or fixed_frame.
	std::optional<Eigen::Isometry3d> camera_guess;

	/// Initial guess of the pattern, can be used to speed up the calibration optimization.
	/// Depending on the value of camera_moving, this pose is defined in either moving_frame or fixed_frame.
	std::optional<Eigen::Isometry3d> pattern_guess;

	/// Directory in which calibration information will be stored (leave empty to disable).
	std::string dump_dir;
};

struct EnsensoCalibrationConfig {
	InitializeCalibrationConfig initialize_config;
	bool store_calibration;
};

DR_PARAM_DEFINE_STRUCT_DECOMPOSITION(EnsensoCalibrationConfig,
(initialize_config,     "InitializeCalibrationConfig",  "", true)
(store_calibration,     "bool",                         "", false)
);

DR_PARAM_DEFINE_STRUCT_DECOMPOSITION(InitializeCalibrationConfig,
(camera_moving, "bool"  , "", true)
(moving_frame , "string", "", true)
(fixed_frame  , "string", "", true)
(camera_guess , "pose"  , "", true)
(pattern_guess, "pose"  , "", true)
(dump_dir     , "string", "", true)
);

