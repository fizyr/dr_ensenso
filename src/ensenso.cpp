#include "ensenso.hpp"
#include "eigen.hpp"
#include "util.hpp"
#include "opencv.hpp"
#include "pcl.hpp"

#include <stdexcept>
#include <fstream>

#include <json/json.h>

namespace dr {

NxLibInitGuard::NxLibInitGuard() {
	try {
		nxLibInitialize();
	} catch (NxLibException const & e) {
		throw NxError(e);
	}
}

NxLibInitGuard::~NxLibInitGuard() {
	if (!moved_) nxLibFinalize();
}

NxLibInitGuard::NxLibInitGuard(NxLibInitGuard && other) {
	other.moved_ = true;
}

NxLibInitGuard & NxLibInitGuard::operator=(NxLibInitGuard && other) {
	other.moved_ = true;
	return *this;
}

NxLibItem imageNode(NxLibItem stereo, std::optional<NxLibItem> monocular, ImageType type) {
	switch (type) {
		case ImageType::stereo_raw_left:             return stereo[itmImages][itmRaw][itmLeft];
		case ImageType::stereo_raw_right:            return stereo[itmImages][itmRaw][itmRight];
		case ImageType::stereo_rectified_left:       return stereo[itmImages][itmRectified][itmLeft];
		case ImageType::stereo_rectified_right:      return stereo[itmImages][itmRectified][itmRight];
		case ImageType::disparity:                   return stereo[itmImages][itmDisparityMap];
		case ImageType::monocular_raw:               return (*monocular)[itmImages][itmRaw];
		case ImageType::monocular_rectified:         return (*monocular)[itmImages][itmRectified];
		case ImageType::monocular_overlay:           return (*monocular)[itmImages][itmWithOverlay];
	}
	throw std::runtime_error("Failed to get image node: unknown image type: " + std::to_string(int(type)));
}

Ensenso::Ensenso(std::string serial, bool connect_monocular, NxLibInitToken token) : init_token_{std::move(token)} {
	if (serial == "") {
		// Try to find a stereo camera.
		std::optional<NxLibItem> camera = openCameraByType(valStereo);
		if (!camera) throw std::runtime_error("Failed to open any Ensenso camera.");
		stereo_node = *camera;
	} else {
		// Open the requested camera.
		std::optional<NxLibItem> camera = openCameraBySerial(serial);
		if (!camera) throw std::runtime_error("Could not open an Ensenso camera with serial " + serial + ".");
		stereo_node = *camera;
	}

	// Get the linked monocular camera.
	if (connect_monocular) {
		monocular_node = openCameraByLink(serialNumber());
		if (!monocular_node) throw std::runtime_error("Failed to open linked monocular camera.");
	}
}

Ensenso::~Ensenso() {
	if (!init_token_) return;
	executeNx(NxLibCommand(cmdClose));
}

std::string Ensenso::serialNumber() const {
	return getNx<std::string>(stereo_node[itmSerialNumber]);
}

std::string Ensenso::monocularSerialNumber() const {
	return monocular_node ? getNx<std::string>(monocular_node.value()[itmSerialNumber]) : "";
}

bool Ensenso::loadParameters(std::string const parameters_file, bool entire_tree) {
	std::ifstream file;
	file.open(parameters_file);

	if (!file.good()) {
		return false;
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	Json::Value root;
	file >> root;

	if (entire_tree) {
		if (!root.isMember("Parameters")) {
			// Requested to load an entire tree, but the input did not contain an entire tree.
			return false;
		}

		setNxJson(stereo_node, Json::writeString(Json::StreamWriterBuilder(), root));
	} else {
		if (root.isMember("Parameters")) {
			// Input was a complete tree, select only the parameters subtree.
			root = root["Parameters"];
		}

		setNxJson(stereo_node[itmParameters], Json::writeString(Json::StreamWriterBuilder(), root));
	}

	return true;
}

bool Ensenso::loadMonocularParameters(std::string const parameters_file, bool entire_tree) {
	if (!monocular_node) throw std::runtime_error("No monocular camera found. Can not load monocular camara parameters.");

	std::ifstream file;
	file.open(parameters_file);

	if (!file.good()) {
		return false;
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	Json::Value root;
	file >> root;

	if (entire_tree) {
		if (!root.isMember("Parameters")) {
			// Requested to load an entire tree, but the input did not contain an entire tree.
			return false;
		}

		setNxJson(monocular_node.value(), Json::writeString(Json::StreamWriterBuilder(), root));
	} else {
		if (root.isMember("Parameters")) {
			// Input was a complete tree, select only the parameters subtree.
			root = root["Parameters"];
		}

		setNxJson(monocular_node.value()[itmParameters], Json::writeString(Json::StreamWriterBuilder(), root));
	}

	return true;
}

void Ensenso::loadMonocularUeyeParameters(std::string const parameters_file) {
	if (!monocular_node) throw std::runtime_error("No monocular camera found. Can not load monocular camera UEye parameters.");
	NxLibCommand command(cmdLoadUEyeParameterSet);
	setNx(command.parameters()[itmFilename], parameters_file);
	executeNx(command);
}

bool Ensenso::hasFlexView() const {
	return stereo_node[itmParameters][itmCapture][itmFlexView].exists();
}

int Ensenso::flexView() const {
	try {
		// in case FlexView = false, getting the int value gives an error
		return getNx<int>(stereo_node[itmParameters][itmCapture][itmFlexView]);
	} catch (NxError const & e) {
		return -1;
	}
}

void Ensenso::setFlexView(int value) {
	setNx(stereo_node[itmParameters][itmCapture][itmFlexView], value);
}

bool Ensenso::hasFrontLight() const {
	return stereo_node[itmParameters][itmCapture][itmFrontLight].exists();
}

std::optional<bool> Ensenso::frontLight() {
	NxLibItem item = stereo_node[itmParameters][itmCapture][itmFrontLight];
	if (!item.exists()) return std::nullopt;
	return getNx<bool>(item);
}

void Ensenso::setFrontLight(bool state) {
	setNx(stereo_node[itmParameters][itmCapture][itmFrontLight], state);
}

bool Ensenso::projector() {
	return getNx<bool>(stereo_node[itmParameters][itmCapture][itmProjector]);
}

void Ensenso::setProjector(bool state) {
	setNx(stereo_node[itmParameters][itmCapture][itmProjector], state);
}

void Ensenso::setDisparityRegionOfInterest(cv::Rect const & roi) {
	if (roi.area() == 0) {
		setNx(stereo_node[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest], false);

		if (stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest].exists()) {
			stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest].erase();
		}
	} else {
		setNx(stereo_node[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest],          true);
		setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0],     roi.tl().x);
		setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1],     roi.tl().y);
		setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0], roi.br().x);
		setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1], roi.br().y);
	}
}

bool Ensenso::trigger(bool stereo, bool monocular) const {
	monocular = monocular && monocular_node;

	NxLibCommand command(cmdTrigger);
	if (stereo) setNx(command.parameters()[itmCameras][0], serialNumber());
	if (monocular) setNx(command.parameters()[itmCameras][stereo ? 1 : 0], getNx<std::string>(monocular_node.value()[itmSerialNumber]));
	executeNx(command);

	if (stereo && !getNx<bool>(command.result()[serialNumber()][itmTriggered])) return false;
	if (monocular && !getNx<bool>(command.result()[monocularSerialNumber()][itmTriggered])) return false;
	return true;
}

bool Ensenso::retrieve(bool trigger, unsigned int timeout, bool stereo, bool monocular) const {
	monocular = monocular && monocular_node;

	// nothing to do?
	if (!stereo && !monocular)
		return true;

	NxLibCommand command(trigger ? cmdCapture : cmdRetrieve);
	setNx(command.parameters()[itmTimeout], int(timeout));
	if (stereo) setNx(command.parameters()[itmCameras][0], serialNumber());
	if (monocular) setNx(command.parameters()[itmCameras][stereo ? 1 : 0], getNx<std::string>(monocular_node.value()[itmSerialNumber]));
	executeNx(command);

	if (stereo && !getNx<bool>(command.result()[serialNumber()][itmRetrieved])) return false;
	if (monocular && !getNx<bool>(command.result()[monocularSerialNumber()][itmRetrieved])) return false;
	return true;
}

void Ensenso::rectifyImages(bool stereo, bool monocular) {
	NxLibCommand command(cmdRectifyImages);
	if (stereo) setNx(command.parameters()[itmCameras][0], serialNumber());
	if (monocular) setNx(command.parameters()[itmCameras][stereo ? 1 : 0], monocularSerialNumber());
	executeNx(command);
}

void Ensenso::computeDisparity() {
	NxLibCommand command(cmdComputeDisparityMap);
	setNx(command.parameters()[itmCameras], serialNumber());
	executeNx(command);
}

void Ensenso::computePointCloud() {
	NxLibCommand command(cmdComputePointMap);
	setNx(command.parameters()[itmCameras], serialNumber());
	executeNx(command);
}

void Ensenso::registerPointCloud() {
	NxLibCommand command(cmdRenderPointMap);
	setNx(command.parameters()[itmNear], 1); // distance in millimeters to the camera (clip nothing?)
	setNx(command.parameters()[itmCamera], monocularSerialNumber());
	// gives weird (RenderPointMap) results with OpenGL enabled, so disable
	setNx(root[itmParameters][itmRenderPointMap][itmUseOpenGL], false);
	executeNx(command);
}

cv::Mat Ensenso::loadImage(ImageType type) {
	return toCvMat(imageNode(stereo_node, monocular_node, type));
}

pcl::PointCloud<pcl::PointXYZ> Ensenso::loadPointCloud() {
	// Convert the binary data to a point cloud.
	return toPointCloud(stereo_node[itmImages][itmPointMap]);
}

pcl::PointCloud<pcl::PointXYZ> Ensenso::loadRegisteredPointCloud() {
	return toPointCloud(root[itmImages][itmRenderPointMap]);
}

void Ensenso::discardCalibrationPatterns() {
	executeNx(NxLibCommand(cmdDiscardPatterns));
}

void Ensenso::recordCalibrationPattern(std::string * parameters_dump_info, std::string * result_dump_info) {
	// disable FlexView
	int flex_view = flexView();
	if (flex_view > 0) setFlexView(0);

	// Capture image with front-light.
	setProjector(false);
	if (hasFrontLight()) setFrontLight(true);

	retrieve(true, 1500, true, false);

	if (hasFrontLight()) setFrontLight(false);
	setProjector(true);

	// Find the pattern.
	NxLibCommand command_collect_pattern(cmdCollectPattern);
	setNx(command_collect_pattern.parameters()[itmCameras], serialNumber());
	setNx(command_collect_pattern.parameters()[itmDecodeData], true);

	// Optionally copy the parameters for debugging.
	if (parameters_dump_info) *parameters_dump_info = command_collect_pattern.parameters().asJson(true);

	try {
		executeNx(command_collect_pattern);
	} catch (std::exception const & e) {
		// Optionally copy the result for debugging.
		if (result_dump_info) *result_dump_info = command_collect_pattern.result().asJson(true);
		throw;
	}

	// restore FlexView setting
	if (flex_view > 0) {
		setFlexView(flex_view);
	}

	// Optionally copy the result for debugging.
	if (result_dump_info) *result_dump_info = command_collect_pattern.result().asJson(true);
}

Eigen::Isometry3d Ensenso::detectCalibrationPattern(int const samples, bool ignore_calibration)  {
	discardCalibrationPatterns();

	for (int i = 0; i < samples; ++i) {
		recordCalibrationPattern();
	}

	// Disable FlexView (should not be necessary here, but appears to be necessary for cmdEstimatePatternPose)
	int flex_view = flexView();
	if (flex_view > 0) setFlexView(0);

	// Get the pose of the pattern.
	NxLibCommand command_estimate_pose(cmdEstimatePatternPose);
	executeNx(command_estimate_pose);

	// Restore FlexView setting.
	if (flex_view > 0) {
		setFlexView(flex_view);
	}

	Eigen::Isometry3d result = toEigenIsometry(command_estimate_pose.result()["Patterns"][0][itmPatternPose]);
	result.translation() *= 0.001;

	// Transform back to left stereo lens.
	if (ignore_calibration) {
		std::optional<Eigen::Isometry3d> camera_pose = getWorkspaceCalibration();
		if (camera_pose) {
			result = *camera_pose * result;
		}
	}

	return result;
}

std::string Ensenso::getWorkspaceCalibrationFrame() {
	// Make sure the relevant nxLibItem exists and is non-empty, then return it.
	NxLibItem item = stereo_node[itmLink][itmTarget];
	int error;
	std::string result = item.asString(&error);
	return error ? "" : result;
}

std::optional<Eigen::Isometry3d> Ensenso::getWorkspaceCalibration() {
	// Check if the camera is calibrated.
	if (getWorkspaceCalibrationFrame().empty()) return std::nullopt;

	// convert from mm to m
	Eigen::Isometry3d pose = toEigenIsometry(stereo_node[itmLink]);
	pose.translation() *= 0.001;

	return pose;
}

Ensenso::CalibrationResult Ensenso::computeCalibration(
	std::vector<Eigen::Isometry3d> const & robot_poses,
	bool moving,
	std::optional<Eigen::Isometry3d> const & camera_guess,
	std::optional<Eigen::Isometry3d> const & pattern_guess,
	std::string const & target,
	std::string * parameters_dump_info,
	std::string * result_dump_info
) {
	NxLibCommand calibrate(cmdCalibrateHandEye);

	// camera pose initial guess
	if (camera_guess) {
		Eigen::Isometry3d scaled_camera_guess = *camera_guess;
		scaled_camera_guess.translation() *= 1000;
		setNx(calibrate.parameters()[itmLink], scaled_camera_guess);
	}

	// pattern pose initial guess
	if (pattern_guess) {
		Eigen::Isometry3d scaled_pattern_guess = *pattern_guess;
		scaled_pattern_guess.translation() *= 1000;
		setNx(calibrate.parameters()[itmPatternPose], scaled_pattern_guess);
	}

	// setup (camera in hand / camera fixed)
	setNx(calibrate.parameters()[itmSetup], moving ? valMoving : valFixed);

	// name of the target coordinate system
	if (target != "") {
		setNx(calibrate.parameters()[itmTarget], target);
	}

	// copy robot poses to parameters
	for (size_t i = 0; i < robot_poses.size(); i++) {
		Eigen::Isometry3d scaled_robot_pose = robot_poses[i];
		scaled_robot_pose.translation() *= 1000;
		setNx(calibrate.parameters()[itmTransformations][i], scaled_robot_pose);
	}

	// Optionally copy the parameters for debugging.
	if (parameters_dump_info) *parameters_dump_info = calibrate.parameters().asJson(true);

	try {
		// execute calibration command
		executeNx(calibrate);
	} catch (std::exception const & e) {
		// Optionally copy the result for debugging.
		if (result_dump_info) *result_dump_info = calibrate.result().asJson(true);
		throw;
	}

	// return result (camera pose, pattern pose, iterations, reprojection error)
	Eigen::Isometry3d camera_pose  = toEigenIsometry(stereo_node[itmLink]).inverse(); // "Link" is inverted
	Eigen::Isometry3d pattern_pose = toEigenIsometry(calibrate.result()[itmPatternPose]);
	camera_pose.translation()  *= 0.001;
	pattern_pose.translation() *= 0.001;

	// Optionally copy the result for debugging.
	if (result_dump_info) *result_dump_info = calibrate.result().asJson(true);

	return Ensenso::CalibrationResult{
		camera_pose,
		pattern_pose,
		getNx<int>(calibrate.result()[itmIterations]),
		getNx<double>(calibrate.result()[itmResidual])
	};
}

void Ensenso::setWorkspaceCalibration(Eigen::Isometry3d const & workspace, std::string const & frame_id, Eigen::Isometry3d const & defined_pose, bool store) {
	NxLibCommand command(cmdCalibrateWorkspace);
	setNx(command.parameters()[itmCameras][0], serialNumber());

	// scale to [mm]
	Eigen::Isometry3d workspace_mm = workspace;
	workspace_mm.translation() *= 1000;
	setNx(command.parameters()[itmPatternPose], workspace_mm);

	if (frame_id != "") {
		setNx(command.parameters()[itmTarget], frame_id);
	}

	Eigen::Isometry3d defined_pose_mm = defined_pose;
	defined_pose_mm.translation() *= 1000;
	setNx(command.parameters()[itmDefinedPose], defined_pose_mm);

	executeNx(command);

	if (store) storeWorkspaceCalibration();
}

void Ensenso::clearWorkspaceCalibration(bool store) {
	// Check if the camera is calibrated.
	if (getWorkspaceCalibrationFrame().empty()) return;

	// calling CalibrateWorkspace with no PatternPose and DefinedPose clears the workspace.
	NxLibCommand command(cmdCalibrateWorkspace);
	setNx(command.parameters()[itmCameras][0], serialNumber());
	setNx(command.parameters()[itmTarget], "");
	executeNx(command);

	// clear target name
	// TODO: Can be removed after settings the target parameter above?
	// TODO: Should test that.
	setNx(stereo_node[itmLink][itmTarget], "");

	if (store) storeWorkspaceCalibration();
}

void Ensenso::storeWorkspaceCalibration() {
	NxLibCommand command(cmdStoreCalibration);
	setNx(command.parameters()[itmCameras][0], serialNumber());
	setNx(command.parameters()[itmLink], true);
	executeNx(command);
}


Eigen::Isometry3d Ensenso::getMonocularLink() const {
	// convert from mm to m
	Eigen::Isometry3d pose = toEigenIsometry(monocular_node.value()[itmLink]);
	pose.translation() *= 0.001;

	return pose;
}

}
