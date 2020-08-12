#include "ensenso.hpp"
#include "eigen.hpp"
#include "util.hpp"
#include "opencv.hpp"
#include "pcl.hpp"

#include <stdexcept>
#include <fstream>

#include <fmt/format.h>
#include <json/json.h>

namespace dr {

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

Result<NxLibInitToken> NxLibInitGuard::initNxLib() {
	int error = 0;
	nxLibInitialize(error);
	if (error) {
		// TODO: check if this is the correct error!!
		Result<NxCommandError> nx_error = NxCommandError::getCurrent();
		if (!nx_error) return estd::error("failed to initialize nx library");
		return Error(nx_error->what());
	}

	return NxLibInitGuard::create_shared();
}

Result<NxLibItem> imageNode(NxLibItem stereo, std::optional<NxLibItem> monocular, ImageType type) {
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
	return estd::error("failed to get image node: unknown image type: " + std::to_string(int(type)));
}

Result<Ensenso> Ensenso::open(std::string serial, bool connect_monocular, LogFunction logger, NxLibInitToken token) {
	NxLibItem camera_node;
	std::optional<NxLibItem> monocular_node;

	if (!token) {
		Result<NxLibInitToken> new_token = NxLibInitGuard::initNxLib();
		if (!new_token) return new_token.error();
		token = *new_token;
	}

	if (serial == "") {
		logger("Opening first available Ensenso camera.");
		// Try to find a stereo camera.
		Result<NxLibItem> camera = openCameraByType(valStereo, logger);
		if (!camera) return camera.error().push_description("failed to open any camera");

		camera_node = *camera;
	} else {
		logger(fmt::format("Opening camera with serial {}.", serial));

		// Try to find a stereo camera.
		Result<NxLibItem> camera = openCameraBySerial(serial);
		if (!camera) return camera.error().push_description(fmt::format("failed to open ensenso camera with serial {}", serial));

		camera_node = *camera;
	}

	// Get the linked monocular camera.
	if (connect_monocular) {
		logger("Looking for linked monocular camera.");

		Result<std::string> serial_number = getNx<std::string>(camera_node[itmSerialNumber]);
		if (!serial_number) serial_number.error().push_description("failed to open linked monocular camera");

		// Try to find a monocula camera.
		Result<NxLibItem> camera = openCameraByLink(serial);
		if (!camera) return camera.error().push_description("failed to open linked monocular camera");

		monocular_node = *camera;

		Result<std::string> serial_number_monocular = getNx<std::string>(monocular_node.value()[itmSerialNumber]);
		if (!serial_number_monocular) serial_number_monocular.error().push_description("failed to open linked monocular camera");

		logger(fmt::format("Opened monocular camera with serial {}", *serial_number_monocular));
	}

	return Ensenso(camera_node, monocular_node, token, logger);
}

Ensenso::~Ensenso() {
	if (!init_token_) return;
	executeNx(NxLibCommand(cmdClose));
}

Result<std::string> Ensenso::serialNumber() const {
	return getNx<std::string>(stereo_node[itmSerialNumber]);
}

Result<std::string> Ensenso::serialNumber(NxLibItem const & item) const {
	return getNx<std::string>(item[itmSerialNumber]);
}

Result<std::string> Ensenso::monocularSerialNumber() const {
	return getNx<std::string>(monocular_node.value()[itmSerialNumber]);
}

Result<void> Ensenso::loadParameters(std::string const parameters_file, bool entire_tree) {
	std::ifstream file;
	file.open(parameters_file);

	if (!file.good()) {
		return estd::error("failed to load parameters: file can't be opened");
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	Json::Value root;
	try {
		file >> root;
	} catch (std::exception &e) {
		return estd::error(fmt::format("failed to load parameters: {}", e.what()));
	}

	if (entire_tree) {
		if (!root.isMember("Parameters")) {
			return estd::error("failed to load parameters: requested to load an entire tree, but the input did not contain an entire tree");
		}

		Result<void> set_nx_json_result = setNxJson(stereo_node, Json::writeString(Json::StreamWriterBuilder(), root));
		if (!set_nx_json_result) {
			return set_nx_json_result.error().push_description("failed to load parameters");
		}

	} else {
		if (root.isMember("Parameters")) {
			// Input was a complete tree, select only the parameters subtree.
			root = root["Parameters"];
		}

		Result<void> set_nx_json_result = setNxJson(stereo_node[itmParameters], Json::writeString(Json::StreamWriterBuilder(), root));
		if (!set_nx_json_result) {
			return set_nx_json_result.error().push_description("failed to load parameters");
		}

	}

	return estd::in_place_valid;
}

Result<void> Ensenso::loadMonocularParameters(std::string const parameters_file, bool entire_tree) {
	if (!monocular_node) {
		return estd::error("failed to load json parameters: no monocular camera found");
	}

	std::ifstream file;
	file.open(parameters_file);

	if (!file.good()) {
		return estd::error("failed to load json parameters: file can't be opened");
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	Json::Value root;
	try {
		file >> root;
	} catch (std::exception &e) {
		return estd::error(fmt::format("failed to load json parameters: {}", e.what()));
	}

	if (entire_tree) {
		if (!root.isMember("Parameters")) {
			return estd::error("failed to load json parameters: requested to load an entire tree, but the input did not contain an entire tree");
		}

		Result<void> set_json = setNxJson(monocular_node.value(), Json::writeString(Json::StreamWriterBuilder(), root));
		if (!set_json) {
			return set_json.error().push_description("failed to load json parameters");
		}
	} else {
		if (root.isMember("Parameters")) {
			// Input was a complete tree, select only the parameters subtree.
			root = root["Parameters"];
		}

		Result<void> set_json = setNxJson(monocular_node.value()[itmParameters], Json::writeString(Json::StreamWriterBuilder(), root));
		if (!set_json) {
			return set_json.error().push_description("failed to load json parameters");
		}
	}

	return estd::in_place_valid;
}

Result<void> Ensenso::loadMonocularUeyeParameters(std::string const parameters_file) {
	if (!monocular_node) {
		return estd::error("failed to load ueye parameters: no monocular camera found");
	}
	NxLibCommand command(cmdLoadUEyeParameterSet);

	Result<void> set_filename_param = setNx(command.parameters()[itmFilename], parameters_file);
	if (!set_filename_param) return set_filename_param.error().push_description("failed to load ueye parameters");

	return executeNx(command);
}

bool Ensenso::hasFlexView() const {
	return stereo_node[itmParameters][itmCapture][itmFlexView].exists();
}

Result<int> Ensenso::flexView() const {
	return getNx<int>(stereo_node[itmParameters][itmCapture][itmFlexView]);
}

Result<void> Ensenso::setFlexView(int value) {
	log(fmt::format("Setting flex view to {}", value));
	return setNx(stereo_node[itmParameters][itmCapture][itmFlexView], value);
}

bool Ensenso::hasFrontLight() const {
	return stereo_node[itmParameters][itmCapture][itmFrontLight].exists();
}

Result<bool> Ensenso::frontLight() {
	return getNx<bool>(stereo_node[itmParameters][itmCapture][itmFrontLight]);
}

Result<void> Ensenso::setFrontLight(bool state) {
	log(fmt::format("Turning front light {}", state ? "on" : "off"));

	return setNx(stereo_node[itmParameters][itmCapture][itmFrontLight], state);
}

Result<bool> Ensenso::projector() {
	return getNx<bool>(stereo_node[itmParameters][itmCapture][itmProjector]);
}

Result<void> Ensenso::setProjector(bool state) {
	log(fmt::format("Turning projector {}", state ? "on" : "off"));
	return setNx(stereo_node[itmParameters][itmCapture][itmProjector], state);
}

Result<void> Ensenso::setDisparityRegionOfInterest(cv::Rect const & roi) {
	if (roi.area() == 0) {
		Result<void> reset_disparity = setNx(stereo_node[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest], false);
		if (!reset_disparity) return reset_disparity.error().push_description("failed to set disparity region of interest");

		if (stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest].exists()) {
			stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest].erase();
		}
	} else {
		Result<void> set_disparity = setNx(stereo_node[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest], true);
		if (!set_disparity) return set_disparity.error().push_description("failed to set disparity region of interest");
		Result<void> set_tlx = setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0],     roi.tl().x);
		if (!set_tlx) return set_tlx.error().push_description("failed to set disparity region of interest");
		Result<void> set_tly = setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1],     roi.tl().y);
		if (!set_tly) return set_tly.error().push_description("failed to set disparity region of interest");
		Result<void> set_rbx = setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0], roi.br().x);
		if (!set_tly) return set_tly.error().push_description("failed to set disparity region of interest");
		Result<void> set_rby = setNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1], roi.br().y);
		if (!set_tlx) return set_tlx.error().push_description("failed to set disparity region of interest");
	}
	return estd::in_place_valid;
}

Result<void> Ensenso::trigger(bool stereo, bool monocular) const {
	monocular = monocular && monocular_node;

	NxLibCommand command(cmdTrigger);
	log("Triggering cameras:");

	std::string stereo_serial_number = "";
	std::string mono_serial_number = "";

	if (stereo) {
		Result<std::string> serial_number = serialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to trigger: could not find serial number for stereo camera");
		stereo_serial_number = *serial_number;

		log(fmt::format(" - {}", stereo_serial_number));

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], stereo_serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to trigger: could not set camera parameter");
	}

	if (monocular) {
		Result<std::string> serial_number = serialNumber(*monocular_node);
		if (!serial_number) return serial_number.error().push_description("failed to trigger: could not find monocular serial number");
		mono_serial_number = *serial_number;

		log(fmt::format(" - {}", mono_serial_number));

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][stereo ? 1 : 0], mono_serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to trigger: coudl not set camera parameter");
	}

	Result<void> execute_trigger = executeNx(command);
	if (!execute_trigger) return execute_trigger.error().push_description("failed to execute trigger command");

	if (stereo) {
		Result<bool> get_triggered = getNx<bool>(command.result()[stereo_serial_number][itmTriggered]);
		if (!get_triggered && !*get_triggered) return get_triggered.error().push_description("failed to trigger: stereo camera was not triggered");
	}
	if (monocular) {
		Result<bool> get_triggered = getNx<bool>(command.result()[mono_serial_number][itmTriggered]);
		if (!get_triggered && !*get_triggered) return get_triggered.error().push_description("failed to trigger: monocular camera was not triggered");
	}

	return estd::in_place_valid;
}

Result<void> Ensenso::retrieve(bool trigger, unsigned int timeout, bool stereo, bool monocular) const {
	monocular = monocular && monocular_node;

	// nothing to do?
	if (!stereo && !monocular) return estd::error("failed to retrieve: neither stereo or monucular is specified");

	std::string stereo_serial_number = "";
	std::string mono_serial_number = "";

	NxLibCommand command(trigger ? cmdCapture : cmdRetrieve);

	Result<void> set_timeout_param = setNx(command.parameters()[itmTimeout], int(timeout));
	if (!set_timeout_param) return set_timeout_param.error().push_description("failed to retrieve: could not set timeout parameter");

	if (stereo) {
		Result<std::string> serial_number = serialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to retrieve: could not find stereo camera");
		stereo_serial_number = *serial_number;

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], stereo_serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to retrieve: could not set stereo camera parameter");
	}
	if (monocular) {
		Result<std::string> serial_number = serialNumber(*monocular_node);
		if (!serial_number) return serial_number.error().push_description("failed to retrieve: could not find monocular camera");
		mono_serial_number = *serial_number;

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][stereo ? 1 : 0], mono_serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to retrieve: could not set monocular camera parameter");
	}

	Result<void> execute_retrieve = executeNx(command);
	if (!execute_retrieve) return execute_retrieve.error().push_description("failed to execute retrieve command");

	if (stereo) {
		Result<bool> get_retrieved = getNx<bool>(command.result()[stereo_serial_number][itmRetrieved]);
		if (!get_retrieved) return get_retrieved.error().push_description("failed to retrieve stereo results");
	}
	if (monocular) {
		Result<bool> get_retrieved = getNx<bool>(command.result()[mono_serial_number][itmRetrieved]);
		if (!get_retrieved) return get_retrieved.error().push_description("failed to retrieve monocular results");
	}

	return estd::in_place_valid;
}

Result<void> Ensenso::rectifyImages(bool stereo, bool monocular) {
	NxLibCommand command(cmdRectifyImages);
	if (stereo) {
		Result<std::string> serial_number = serialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to rectify stereo camera");

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to rectify stereo camera");
	}
	if (monocular) {
		Result<std::string> serial_number = monocularSerialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to rectify monocular camera");

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][stereo ? 1 : 0], *serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to rectify monocular camera");
	}

	return executeNx(command);
}

Result<void> Ensenso::computeDisparity() {
	NxLibCommand command(cmdComputeDisparityMap);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to compute disparity: could not find camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to compute disparity: could not set camera parameter");

	return executeNx(command);
}

Result<void> Ensenso::computePointCloud() {
	NxLibCommand command(cmdComputePointMap);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to compute point cloud: could not find camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to compute point cloud: could not set camera parameter");

	return executeNx(command);
}

Result<void> Ensenso::registerPointCloud() {
	NxLibCommand command(cmdRenderPointMap);

	Result<std::string> serial_number = monocularSerialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to register point cloud: could not find monocular camera");

	Result<void> set_near_param = setNx(command.parameters()[itmNear], 1); // distance in millimeters to the camera (clip nothing?)
	if (!set_near_param) return set_near_param.error().push_description("failed to register point cloud: could not set near parameter");

	Result<void> set_camera_param = setNx(command.parameters()[itmCamera], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to register point cloud: could not set camera parameter");

	// gives weird (RenderPointMap) results with OpenGL enabled, so disable
	Result<void> set_use_open_gl_param = setNx(root[itmDefaultParameters][itmRenderPointMap][itmUseOpenGL], false);
	if (!set_use_open_gl_param) return set_use_open_gl_param.error().push_description("failed to register point cloud: could not set 'use open gl' parameter");

	return executeNx(command);
}

Result<cv::Rect> Ensenso::getRoi() {
	Result<int> tlx = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0]);
	if (!tlx) return tlx.error();

	Result<int> tly = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1]);
	if (!tly) return tly.error();

	// As opencv requires exclusive right and bottom boundaries, we increment by 1.
	Result<int> rbx = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0]);
	if (!rbx) return rbx.error();

	Result<int> rby = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1]);
	if (!rby) return rby.error();

	return cv::Rect{cv::Point2i{*tlx, *tly}, cv::Point2i{*rbx + 1, *rby + 1}};
}

std::optional<cv::Rect> Ensenso::getOptionalRoi() {
	Result<cv::Rect> get_roi_result = getRoi();
	if (!get_roi_result) return std::nullopt;
	return *get_roi_result;
}

Result<cv::Mat> Ensenso::loadImage(ImageType type, bool crop_to_roi) {
	Result<NxLibItem> image_node = imageNode(stereo_node, monocular_node, type);
	if (!image_node) {
		return image_node.error().push_description("failed to load image");
	}

	return toCvMat(*image_node, crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<void> Ensenso::loadImage(
	ImageType type,
	std::uint8_t * pointer,
	std::size_t width,
	std::size_t height,
	int cv_type,
	bool crop_to_roi
) {
	Result<NxLibItem> image_node = imageNode(stereo_node, monocular_node, type);
	if (!image_node) {
		return image_node.error().push_description("failed to load image");
	}

	return toCvMat(
		*image_node,
		pointer,
		width,
		height,
		cv_type,
		crop_to_roi ? getOptionalRoi() : std::nullopt
	);
}

Result<pcl::PointCloud<pcl::PointXYZ>> Ensenso::loadPointCloud(bool crop_to_roi) {
	// Convert the binary data to a point cloud.
	return toPointCloud(stereo_node[itmImages][itmPointMap], crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<void> Ensenso::loadPointCloudToBuffer(float* buf, std::size_t width, std::size_t height, bool crop_to_roi) {
	return pointCloudToBuffer(stereo_node[itmImages][itmPointMap], "", buf, width, height, crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<pcl::PointCloud<pcl::PointXYZ>> Ensenso::loadRegisteredPointCloud(bool crop_to_roi) {
	return toPointCloud(root[itmImages][itmRenderPointMap], crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<void> Ensenso::loadRegisteredPointCloudToBuffer(float* buf, std::size_t width, std::size_t height, bool crop_to_roi) {
	return pointCloudToBuffer(root[itmImages][itmRenderPointMap], "", buf, width, height, crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<void> Ensenso::discardCalibrationPatterns() {
	return executeNx(NxLibCommand(cmdDiscardPatterns));
}

Result<void> Ensenso::recordCalibrationPattern(std::string * parameters_dump_info, std::string * result_dump_info) {
	// disable FlexView
	Result<int> flex_view = flexView();
	if (!flex_view) return flex_view.error();
	if (*flex_view > 0) setFlexView(0);

	// Capture image with front-light.
	setProjector(false);
	if (hasFrontLight()) setFrontLight(true);

	retrieve(true, 1500, true, false);

	if (hasFrontLight()) setFrontLight(false);
	setProjector(true);

	Result<std::string> serial_number_result = serialNumber();
	if (!serial_number_result) return serial_number_result.error().push_description("failed to record calibration pattern: could not find serial number for stereo camera");

	// Find the pattern.
	NxLibCommand command_collect_pattern(cmdCollectPattern);

	Result<void> set_camera_param = setNx(command_collect_pattern.parameters()[itmCameras], *serial_number_result);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to record calibration pattern: failed to set camera parameter");

	Result<void> set_decode_data_param = setNx(command_collect_pattern.parameters()[itmDecodeData], true);
	if (!set_decode_data_param) return set_decode_data_param.error().push_description("failed to record calibration pattern: failed to set decode data parameter");

	// Optionally copy the parameters for debugging.
	if (parameters_dump_info) *parameters_dump_info = command_collect_pattern.parameters().asJson(true);

	Result<void> execute_collect_pattern = executeNx(command_collect_pattern);
	if (!execute_collect_pattern) {
		// TODO:Why do this???
		// Optionally copy the result for debugging.
		if (result_dump_info) *result_dump_info = command_collect_pattern.result().asJson(true);
		return execute_collect_pattern.error().push_description("failed to execute collect pattern command");
	}

	// restore FlexView setting
	if (*flex_view > 0) {
		setFlexView(*flex_view);
	}

	// Optionally copy the result for debugging.
	if (result_dump_info) *result_dump_info = command_collect_pattern.result().asJson(true);

	return estd::in_place_valid;
}

Result<Eigen::Isometry3d> Ensenso::detectCalibrationPattern(int const samples, bool ignore_calibration)  {
	Result<void> discard_calibration_patterns = discardCalibrationPatterns();
	if (!discard_calibration_patterns) {
		return discard_calibration_patterns.error().push_description("failed to detect calibration pattern");
	}

	for (int i = 0; i < samples; ++i) {
		Result<void> record_calibration_pattern = recordCalibrationPattern();
		if (!record_calibration_pattern) {
			return record_calibration_pattern.error().push_description(fmt::format("failed to detect calibration pattern: record calibration pattern failed at sample {}", i));
		}
	}

	// Disable FlexView (should not be necessary here, but appears to be necessary for cmdEstimatePatternPose)
	Result<int> flex_view = flexView();
	if (!flex_view) return flex_view.error();
	if (*flex_view > 0) setFlexView(0);

	// Get the pose of the pattern.
	NxLibCommand command_estimate_pose(cmdEstimatePatternPose);
	Result<void> execute_estimate = executeNx(command_estimate_pose);
	if (!execute_estimate) return execute_estimate.error().push_description("failed to execute estimate pose command");

	// TODO:To this if execute fails as well??
	// Restore FlexView setting.
	if (*flex_view > 0) {
		setFlexView(*flex_view);
	}

	Result<Eigen::Isometry3d> eigen_isometry = toEigenIsometry(command_estimate_pose.result()["Patterns"][0][itmPatternPose]);
	if (!eigen_isometry) return eigen_isometry.error().push_description("failed to detect calibration pattern: eigen isometry not found");

	eigen_isometry->translation() *= 0.001;

	// Transform back to left stereo lens.
	if (ignore_calibration) {
		Result<Eigen::Isometry3d> camera_pose = getWorkspaceCalibration();
		if (camera_pose) {
			*eigen_isometry = *camera_pose * *eigen_isometry;
		}
	}

	return *eigen_isometry;
}

std::string Ensenso::getWorkspaceCalibrationFrame() {
	// TODO: implement return getNx<std::string>(stereo_node[itmLink][itmTarget]) . This could break code somewhere else if we returned an error here.

	// Make sure the relevant nxLibItem exists and is non-empty, then return it.
	NxLibItem item = stereo_node[itmLink][itmTarget];
	int error;
	std::string result = item.asString(&error);
	return error ? "" : result;
}

Result<Eigen::Isometry3d> Ensenso::getWorkspaceCalibration() {
	// Check if the camera is calibrated.
	if (getWorkspaceCalibrationFrame().empty()) return estd::error("failed to retrieve workspace calibration: workspace calibration frame not set");

	// convert from mm to m
	Result<Eigen::Isometry3d> pose = toEigenIsometry(stereo_node[itmLink]);
	if (!pose) return pose.error().push_description("failed to retrieve workspace calibration: pose not found");
	pose->translation() *= 0.001;

	return pose;
}

Result<Ensenso::CalibrationResult> Ensenso::computeCalibration(
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
	Result<void> set_setup_param = setNx(calibrate.parameters()[itmSetup], moving ? valMoving : valFixed);
	if (!set_setup_param) return set_setup_param.error().push_description("failed to compute calibration: could not set setup parameter");

	// name of the target coordinate system
	if (target != "") {
		Result<void> set_target_param = setNx(calibrate.parameters()[itmTarget], target);
		if (!set_target_param) return set_target_param.error().push_description("failed to compute calibration: could not set target parameter");
	}

	// copy robot poses to parameters
	for (size_t i = 0; i < robot_poses.size(); i++) {
		Eigen::Isometry3d scaled_robot_pose = robot_poses[i];
		scaled_robot_pose.translation() *= 1000;
		Result<void> set_transform_param = setNx(calibrate.parameters()[itmTransformations][i], scaled_robot_pose);
		if (!set_transform_param) {
			return set_transform_param.error().push_description(fmt::format("failed to compute calibration: could not set transform parameter {} ", i));
		}
	}

	// Optionally copy the parameters for debugging.
	if (parameters_dump_info) *parameters_dump_info = calibrate.parameters().asJson(true);

	Result<void> execute_calibrate = executeNx(calibrate);
	if (!execute_calibrate) {
		if (result_dump_info) *result_dump_info = calibrate.result().asJson(true);
		return execute_calibrate.error().push_description("failed to execute calibrate command");
	}

	// return result (camera pose, pattern pose, iterations, reprojection error)
	Result<Eigen::Isometry3d> camera_pose  = toEigenIsometry(stereo_node[itmLink]);
	if (!camera_pose) camera_pose.error().push_description("failed to compute calibration: camera pose not found");
	camera_pose->inverse(); // "Link" is inverted

	Result<Eigen::Isometry3d> pattern_pose = toEigenIsometry(calibrate.result()[itmPatternPose]);
	if (!pattern_pose) pattern_pose.error().push_description("failed to compute calibration: pattern pose not found");

	camera_pose->translation()  *= 0.001;
	pattern_pose->translation() *= 0.001;

	// Optionally copy the result for debugging.
	if (result_dump_info) *result_dump_info = calibrate.result().asJson(true);

	Result<int> iterations = getNx<int>(calibrate.result()[itmIterations]);
	if (!iterations) return iterations.error().push_description("failed to compute calibration: iterations not found");

	Result<double> residual = getNx<double>(calibrate.result()[itmResidual]);
	if (!residual) return residual.error().push_description("failed to compute calibration: residual not found");

	return Ensenso::CalibrationResult{
		*camera_pose,
		*pattern_pose,
		*iterations,
		*residual
	};
}

Result<void> Ensenso::setWorkspaceCalibration(Eigen::Isometry3d const & workspace, std::string const & frame_id, Eigen::Isometry3d const & defined_pose, bool store) {
	NxLibCommand command(cmdCalibrateWorkspace);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to set workspace calibration: could not find camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to set workspace calibration: could not set camera parameter");

	// scale to [mm]
	Eigen::Isometry3d workspace_mm = workspace;
	workspace_mm.translation() *= 1000;

	Result<void> set_pattern_pose_param = setNx(command.parameters()[itmPatternPose], workspace_mm);
	if (!set_pattern_pose_param) return set_pattern_pose_param.error().push_description("failed to set workspace calibration: could not set pattern pose parameter");

	if (frame_id != "") {
		Result<void> set_target_param = setNx(command.parameters()[itmTarget], frame_id);
		if (!set_target_param) return set_target_param.error().push_description("failed to set workspace calibration: could not set target parameter");
	}

	Eigen::Isometry3d defined_pose_mm = defined_pose;
	defined_pose_mm.translation() *= 1000;

	Result<void> set_defined_pose_param = setNx(command.parameters()[itmDefinedPose], defined_pose_mm);
	if (!set_defined_pose_param) return set_defined_pose_param.error().push_description("failed to set workspace calibration: could not set defined pose parameter");

	Result<void> execute_calibrate_workspace = executeNx(command);
	if (!execute_calibrate_workspace) return execute_calibrate_workspace.error().push_description("failed to execute calibrate workspace command");

	if (store) storeWorkspaceCalibration();

	return estd::in_place_valid;
}

Result<void> Ensenso::clearWorkspaceCalibration(bool store) {
	// Check if the camera is calibrated.
	// TODO: check if we should return an error here.
	if (getWorkspaceCalibrationFrame().empty()) return estd::error("failed to clear workspace calibration: workspace calibration frame not set");

	// calling CalibrateWorkspace with no PatternPose and DefinedPose clears the workspace.
	NxLibCommand command(cmdCalibrateWorkspace);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to clear workspace calibration: could not find camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to clear workspace calibration: could not set camera parameter");

	Result<void> set_target_param = setNx(command.parameters()[itmTarget], "");
	if (!set_target_param) return set_target_param.error().push_description("failed to clear workspace calibration: could not set target parameter");

	Result<void> execute_calibrate_workspace = executeNx(command);
	if (!execute_calibrate_workspace) return execute_calibrate_workspace.error().push_description("failed to execute calibrate workspace command");

	// clear target name
	// TODO: Can be removed after settings the target parameter above?
	// TODO: Should test that.
	Result<void> set_stereo_node_target = setNx(stereo_node[itmLink][itmTarget], "");;
	if (!set_stereo_node_target) return set_stereo_node_target.error().push_description("failed to clear workspace calibration: could not set stereo_node target");

	if (store) storeWorkspaceCalibration();

	return estd::in_place_valid;
}

Result<void> Ensenso::storeWorkspaceCalibration() {
	NxLibCommand command(cmdStoreCalibration);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to store workspace calibration: could not find camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to store workspace calibration: could not set camera parameter");

	Result<void> set_link_param = setNx(command.parameters()[itmLink], true);
	if (!set_link_param) return set_link_param.error().push_description("failed to store workspace calibration: could not set link parameter");

	return executeNx(command);
}


Result<Eigen::Isometry3d> Ensenso::getMonocularLink() const {
	// convert from mm to m
	Result<Eigen::Isometry3d> pose = toEigenIsometry(monocular_node.value()[itmLink]);
	if (!pose) return pose.error().push_description("failed to get monocular link: could not retrieve pose");

	pose->translation() *= 0.001;

	return pose;
}

Result<Ensenso::CaptureParams> Ensenso::getCaptureParameters(bool crop_to_roi) {
	CaptureParams params;
	if (crop_to_roi) {
		Result<cv::Rect> roi = getRoi();
		if (!roi) return roi.error().push_description("failed to get capture parameters");
		params.stereo_width = roi->width;
		params.stereo_height = roi->height;

		log(fmt::format("Stereo size {}x{}.", params.stereo_width, params.stereo_height));

		if (!monocular_node) return params;

		params.monocular_width = roi->width;
		params.monocular_height = roi->height;
	} else {
		// TODO: Why not use the function:??? getNx<int>(stereo_node[itmSensor]);
		auto stereo_sensor_params = stereo_node[itmSensor];
		auto stereo_size = stereo_sensor_params[itmSize];
		params.stereo_width  = stereo_size[0].asInt();
		params.stereo_height = stereo_size[1].asInt();

		log(fmt::format("Stereo size {}x{}.", params.stereo_width, params.stereo_height));

		if (!monocular_node) return params;

		auto monocular_params = monocular_node.value();
		auto monocular_sensor_params = monocular_params[itmSensor];
		auto monocular_size = monocular_sensor_params[itmSize];
		params.monocular_width  = monocular_size[0].asInt();
		params.monocular_height = monocular_size[1].asInt();
	}

	log(fmt::format("Monocular size {}x{}.", *params.monocular_width, *params.monocular_height));

	return params;
}

}
