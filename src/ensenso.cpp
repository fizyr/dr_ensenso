#include "ensenso.hpp"
#include "eigen.hpp"
#include "util.hpp"
#include "opencv.hpp"
#include "pcl.hpp"

#include <dr_util/timestamp.hpp>
#include <stdexcept>
#include <fstream>

#include <boost/filesystem.hpp>
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
	if (error) return Error("failed to initialize nx lib: " + getNxErrorWithDescription(error));

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
	return Error("failed to get image node: unknown image type: " + std::to_string(int(type)));
}

Result<std::shared_ptr<Ensenso>> Ensenso::openSharedCamera(std::string serial, bool connect_monocular, LogFunction logger, NxLibInitToken token) {
	Result<OpenCameraReturn> open_camera = Ensenso::open(serial, connect_monocular, logger, token);
	if (!open_camera) {
		return open_camera.error();
	}

	return std::make_shared<Ensenso>(Ensenso(std::get<0>(*open_camera),  std::get<1>(*open_camera), std::get<2>(*open_camera), std::get<3>(*open_camera)));
}

Result<Ensenso> Ensenso::openCamera(std::string serial, bool connect_monocular, LogFunction logger, NxLibInitToken token) {
	Result<OpenCameraReturn> open_camera = Ensenso::open(serial, connect_monocular, logger, token);
	if (!open_camera) {
		return open_camera.error();
	}

	return Ensenso(std::get<0>(*open_camera),  std::get<1>(*open_camera), std::get<2>(*open_camera), std::get<3>(*open_camera));
}

Result<OpenCameraReturn> Ensenso::open(std::string serial, bool connect_monocular, LogFunction logger, NxLibInitToken token) {
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
		Result<NxLibItem> camera = openCameraByLink(*serial_number);
		if (!camera) return camera.error().push_description("failed to open linked monocular camera");

		monocular_node = *camera;

		Result<std::string> serial_number_monocular = getNx<std::string>(monocular_node.value()[itmSerialNumber]);
		if (!serial_number_monocular) serial_number_monocular.error().push_description("failed to open linked monocular camera");

		logger(fmt::format("Opened monocular camera with serial {}", *serial_number_monocular));
	}

	return std::make_tuple(camera_node, monocular_node, token, logger);
}

Ensenso::~Ensenso() {
	if (!init_token_) return;
	log("Closing all camera's.");
	//TODO: Check what happens if close is an error.
	Result<void> close = executeNx(NxLibCommand(cmdClose));
	if (!close) {
		logger_(close.error().format());
	}
}

Result<std::string> Ensenso::serialNumber() const {
	return getNx<std::string>(stereo_node[itmSerialNumber]);
}

Result<std::string> Ensenso::monocularSerialNumber() const {
	return getNx<std::string>(monocular_node.value()[itmSerialNumber]);
}

Result<void> Ensenso::loadParameters(std::string const parameters_file, bool entire_tree) {
	std::ifstream file;
	file.open(parameters_file);

	if (!file.good()) {
		return Error("failed to load parameters: file can't be opened");
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	Json::Value root;
	try {
		file >> root;
	} catch (std::exception &e) {
		return Error(fmt::format("failed to load parameters: {}", e.what()));
	}

	if (entire_tree) {
		if (!root.isMember("Parameters")) {
			return Error("failed to load parameters: requested to load an entire tree, but the input did not contain an entire tree");
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
		return Error("failed to load json parameters: no monocular camera found");
	}

	std::ifstream file;
	file.open(parameters_file);

	if (!file.good()) {
		return Error("failed to load json parameters: file can't be opened");
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	Json::Value root;
	try {
		file >> root;
	} catch (std::exception &e) {
		return Error(fmt::format("failed to load json parameters: {}", e.what()));
	}

	if (entire_tree) {
		if (!root.isMember("Parameters")) {
			return Error("failed to load json parameters: requested to load an entire tree, but the input did not contain an entire tree");
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
		return Error("failed to load ueye parameters: no monocular camera found");
	}
	NxLibCommand command(cmdLoadUEyeParameterSet);

	Result<void> set_filename_param = setNx(command.parameters()[itmFilename], parameters_file);
	if (!set_filename_param) return set_filename_param.error().push_description("failed to load ueye parameters");

	return executeNx(command);
}

bool Ensenso::hasFlexView() const {
	Result<bool> flex_view_exists = existsNx(stereo_node[itmParameters][itmCapture][itmFlexView]);
	if (!flex_view_exists) {
		return false;
	}
	return *flex_view_exists;
}

int Ensenso::flexView() const {
	// in case FlexView = false, getting the int value gives an error
	Result<int> flexview = getNx<int>(stereo_node[itmParameters][itmCapture][itmFlexView]);
	if (!flexview) {
		return -1;
	}
	return *flexview;
}

Result<void> Ensenso::setFlexView(int value) {
	log(fmt::format("Setting flex view to {}", value));
	return setNx(stereo_node[itmParameters][itmCapture][itmFlexView], value);
}

Result<void> Ensenso::enableCuda(std::uint32_t device) {
	// Check if cuda is available.
	if (!root[itmCUDA][itmAvailable].asBool()) return Error{"cuda not available"};

	// Check if the selected device is available.
	std::uint32_t num_devices = root[itmCUDA][itmDevices].count();
	if (device >= num_devices) return Error{fmt::format("selected device {} exceeds maximum device id, {}", device, num_devices)};

	// Set cuda device.
	root[itmCUDA][itmDevice] = int(device);

	// Enable cuda.
	root[itmCUDA][itmEnabled] = true;
	return estd::in_place_valid;
}

bool Ensenso::hasFrontLight() const {
	Result<bool> front_light_exists = existsNx(stereo_node[itmParameters][itmCapture][itmFrontLight]);
	if (!front_light_exists) {
		return false;
	}
	return *front_light_exists;
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

		Result<bool> aoi_exists = existsNx(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest]);
		if (!aoi_exists || *aoi_exists) {
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
		if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");
		stereo_serial_number = *serial_number;

		log(fmt::format(" - {}", stereo_serial_number));

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], stereo_serial_number);

		if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial number on trigger command");
	}

	if (monocular) {
		Result<std::string> serial_number = monocularSerialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to determine monocular camera serial number");
		mono_serial_number = *serial_number;

		log(fmt::format(" - {}", mono_serial_number));

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][stereo ? 1 : 0], mono_serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to configure monocular camera serial number on trigger command");
	}

	Result<void> execute_trigger = executeNx(command);
	if (!execute_trigger) return execute_trigger.error();

	if (stereo) {
		Result<bool> get_triggered = getNx<bool>(command.result()[stereo_serial_number][itmTriggered]);
		if (!get_triggered && !*get_triggered) return get_triggered.error().push_description("failed to trigger stereo camera");
	}
	if (monocular) {
		Result<bool> get_triggered = getNx<bool>(command.result()[mono_serial_number][itmTriggered]);
		if (!get_triggered && !*get_triggered) return get_triggered.error().push_description("failed to trigger monocular camera");
	}

	return estd::in_place_valid;
}

Result<void> Ensenso::retrieve(bool trigger, unsigned int timeout, bool stereo, bool monocular) const {
	monocular = monocular && monocular_node;

	// nothing to do?
	if (!stereo && !monocular) return Error("failed to retrieve: neither stereo or monucular is specified");

	std::string stereo_serial_number = "";
	std::string mono_serial_number = "";

	NxLibCommand command(trigger ? cmdCapture : cmdRetrieve);

	Result<void> set_timeout_param = setNx(command.parameters()[itmTimeout], int(timeout));
	if (!set_timeout_param) return set_timeout_param.error().push_description("failed set timeout for retrieving camera data");

	if (stereo) {
		Result<std::string> serial_number = serialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");
		stereo_serial_number = *serial_number;

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], stereo_serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial number on retrieve command");
	}
	if (monocular) {
		Result<std::string> serial_number = monocularSerialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to determine monocular camera serial number");
		mono_serial_number = *serial_number;

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][stereo ? 1 : 0], mono_serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to configure monocular camera serial number on retrieve command");
	}

	Result<void> execute_retrieve = executeNx(command);
	if (!execute_retrieve) return execute_retrieve.error();

	if (stereo) {
		Result<bool> get_retrieved = getNx<bool>(command.result()[stereo_serial_number][itmRetrieved]);
		if (!get_retrieved && !*get_retrieved) return get_retrieved.error().push_description("failed to retrieve stereo camera data");
	}
	if (monocular) {
		Result<bool> get_retrieved = getNx<bool>(command.result()[mono_serial_number][itmRetrieved]);
		if (!get_retrieved && !*get_retrieved) return get_retrieved.error().push_description("failed to retrieve monocular camera data");
	}

	return estd::in_place_valid;
}

Result<void> Ensenso::rectifyImages(bool stereo, bool monocular) {
	NxLibCommand command(cmdRectifyImages);
	if (stereo) {
		Result<std::string> serial_number = serialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial number on rectify command");
	}
	if (monocular) {
		Result<std::string> serial_number = monocularSerialNumber();
		if (!serial_number) return serial_number.error().push_description("failed to determine monocular camera serial number");

		Result<void> set_camera_param = setNx(command.parameters()[itmCameras][stereo ? 1 : 0], *serial_number);
		if (!set_camera_param) return set_camera_param.error().push_description("failed to configure monocular camera serial number on rectify command");
	}

	return executeNx(command);
}

Result<void> Ensenso::computeDisparity() {
	NxLibCommand command(cmdComputeDisparityMap);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial number on compute disparity command");

	return executeNx(command);
}

Result<void> Ensenso::computePointCloud() {
	NxLibCommand command(cmdComputePointMap);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial number on compute point cloud command");

	return executeNx(command);
}

Result<void> Ensenso::registerPointCloud(bool use_open_gl) {
	if (!monocular_node) return Error("failed to register point cloud: monocular camera is not attached");

	NxLibCommand command(cmdRenderPointMap);

	Result<std::string> serial_number = monocularSerialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to determine monocular camera serial number");

	Result<void> set_near_param = setNx(command.parameters()[itmNear], 1); // distance in millimeters to the camera (clip nothing?)
	if (!set_near_param) return set_near_param.error().push_description("failed to configure item near on compute disparity command");

	Result<void> set_camera_param = setNx(command.parameters()[itmCamera], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to configure monocular camera serial number on compute disparity command");

	// OpenGL has been disabled since at least 2016, but it has been causing strange shadowing effects.
	// See https://github.com/fizyr/dr_ensenso/pull/22 for more information.
	Result<void> set_use_open_gl_param = setNx(root[itmDefaultParameters][itmRenderPointMap][itmUseOpenGL], use_open_gl);
	if (!set_use_open_gl_param) return set_use_open_gl_param.error().push_description("failed to configure use open gl on compute disparity command");

	return executeNx(command);
}

Result<cv::Rect> Ensenso::getRoi() {
	Result<int> tlx = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0]);
	if (!tlx) return tlx.error();

	Result<int> tly = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1]);
	if (!tly) return tly.error();

	Result<int> rbx = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0]);
	if (!rbx) return rbx.error();

	Result<int> rby = getNx<int>(stereo_node[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1]);
	if (!rby) return rby.error();

	// As opencv requires exclusive right and bottom boundaries, we increment by 1.
	return cv::Rect{cv::Point2i{*tlx, *tly}, cv::Point2i{*rbx + 1, *rby + 1}};
}

std::optional<cv::Rect> Ensenso::getOptionalRoi() {
	Result<cv::Rect> get_roi_result = getRoi();
	if (!get_roi_result) return std::nullopt;
	return *get_roi_result;
}

Result<cv::Mat> Ensenso::loadImage(ImageType type, bool crop_to_roi) {
	Result<NxLibItem> image_node = imageNode(stereo_node, monocular_node, type);
	if (!image_node) return image_node.error().push_description("failed to load image");

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
	if (!image_node) return image_node.error().push_description("failed to load image");

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
	return pointCloudToBuffer(stereo_node[itmImages][itmPointMap], buf, width, height, crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<pcl::PointCloud<pcl::PointXYZ>> Ensenso::loadRegisteredPointCloud(bool crop_to_roi) {
	return toPointCloud(root[itmImages][itmRenderPointMap], crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<void> Ensenso::loadRegisteredPointCloudToBuffer(float* buf, std::size_t width, std::size_t height, bool crop_to_roi) {
	return pointCloudToBuffer(root[itmImages][itmRenderPointMap], buf, width, height, crop_to_roi ? getOptionalRoi() : std::nullopt);
}

Result<void> Ensenso::discardCalibrationPatterns() {
	return executeNx(NxLibCommand(cmdDiscardPatterns));
}

Result<void> Ensenso::recordCalibrationPattern(std::string * parameters_dump_info, std::string * result_dump_info) {
	// disable FlexView
	Result<int> flex_view = flexView();
	if (!flex_view) return flex_view.error();
	if (*flex_view > 0) {
		if (Error err = setFlexView(0).error_or()) return err.push_description("failed to disable FlexView");
	}

	// Capture image with front-light.
	if (Error err = setProjector(false).error_or()) return err.push_description("failed to set projector state before image acquisition");

	if (hasFrontLight()) {
		if (Error err = setFrontLight(true).error_or()) return err.push_description("failed to set front light state before image acquisition");
	}

	if (Error err = retrieve(true, 1500, true, false).error_or()) return err.push_description("failed to retrieve image");

	if (hasFrontLight()) {
		if (Error err = setFrontLight(false).error_or()) return err.push_description("failed to set front light state after image acquisition");
	}

	if (Error err = setProjector(true).error_or()) return err.push_description("failed to set projector state after image acquisition");



	Result<std::string> serial_number_result = serialNumber();
	if (!serial_number_result) return serial_number_result.error().push_description("failed to determine stereo camera serial number");

	// Find the pattern.
	NxLibCommand command_collect_pattern(cmdCollectPattern);

	Result<void> set_camera_param = setNx(command_collect_pattern.parameters()[itmCameras], *serial_number_result);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial number on record calibration command");

	Result<void> set_decode_data_param = setNx(command_collect_pattern.parameters()[itmDecodeData], true);
	if (!set_decode_data_param) return set_decode_data_param.error().push_description("failed to configure stereo camera serial number on record calibration command");

	// Optionally copy the parameters for debugging.
	if (parameters_dump_info) *parameters_dump_info = command_collect_pattern.parameters().asJson(true);

	Result<void> execute_collect_pattern = executeNx(command_collect_pattern);
	if (!execute_collect_pattern) {
		// TODO:Why do this???
		// Optionally copy the result for debugging.
		if (result_dump_info) *result_dump_info = command_collect_pattern.result().asJson(true);
		return execute_collect_pattern.error();
	}

	// restore FlexView setting
	if (*flex_view > 0) {
		if (Error err = setFlexView(*flex_view).error_or()) return err.push_description("failed to restore FlexView settings");
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
			return record_calibration_pattern.error().push_description("failed to detect calibration pattern");
		}
	}

	// Disable FlexView (should not be necessary here, but appears to be necessary for cmdEstimatePatternPose)
	Result<int> flex_view = flexView();
	if (!flex_view) return flex_view.error();
	if (*flex_view > 0) {
		Result<void> disable_flex_view_result = setFlexView(0);
		if (!disable_flex_view_result) return disable_flex_view_result.error().push_description("failed to disable FlexView");
	}

	// Get the pose of the pattern.
	NxLibCommand command_estimate_pose(cmdEstimatePatternPose);
	Result<void> execute_estimate = executeNx(command_estimate_pose);
	if (!execute_estimate) return execute_estimate.error();

	// Restore FlexView setting.
	if (*flex_view > 0) {
		Result<void> set_flex_view_result = setFlexView(*flex_view);
		if (!set_flex_view_result) return set_flex_view_result.error().push_description("failed to restore FlexView settings");
	}

	Result<Eigen::Isometry3d> eigen_isometry = toEigenIsometry(command_estimate_pose.result()["Patterns"][0][itmPatternPose]);
	if (!eigen_isometry) return eigen_isometry.error().push_description("failed to detect calibration pattern");

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
	// Make sure the relevant nxLibItem exists and is non-empty, then return it.
	NxLibItem item = stereo_node[itmLink][itmTarget];
	int error;
	std::string result = item.asString(&error);
	return error ? "" : result;
}

Result<Eigen::Isometry3d> Ensenso::getWorkspaceCalibration() {
	// Check if the camera is calibrated.
	if (getWorkspaceCalibrationFrame().empty()) return Error("failed to retrieve workspace calibration: workspace calibration frame not set");

	// convert from mm to m
	Result<Eigen::Isometry3d> pose = toEigenIsometry(stereo_node[itmLink]);
	if (!pose) return pose.error().push_description("failed to retrieve workspace calibration");
	pose = pose->inverse(); // "Link" is the pose of the world w.r.t. the camera.
	pose->translation() *= 0.001;

	return pose;
}

Result<Ensenso::CalibrationResult> Ensenso::computeCalibration(
	std::vector<Eigen::Isometry3d> const & robot_poses,
	bool moving,
	std::optional<Eigen::Isometry3d> const & camera_guess,
	std::optional<Eigen::Isometry3d> const & pattern_guess,
	std::optional<std::array<bool, 3>> const & translation_camera_fixed,
	std::optional<std::array<bool, 3>> const & rotation_camera_fixed,
	std::optional<std::array<bool, 3>> const & translation_pattern_fixed,
	std::optional<std::array<bool, 3>> const & rotation_pattern_fixed,
	std::string const & target,
	std::string * parameters_dump_info,
	std::string * result_dump_info
) {
	NxLibCommand calibrate(cmdCalibrateHandEye);

	// camera pose initial guess
	if (camera_guess) {
		Eigen::Isometry3d scaled_camera_guess = *camera_guess;
		scaled_camera_guess.translation() *= 1000;
		if (Error err = setNx(calibrate.parameters()[itmLink], scaled_camera_guess).error_or())
			return err.push_description("failed to configure camera pose initial guess on compute calibration command");
	}

	// pattern pose initial guess
	if (pattern_guess) {
		Eigen::Isometry3d scaled_pattern_guess = *pattern_guess;
		scaled_pattern_guess.translation() *= 1000;
		if (Error err = setNx(calibrate.parameters()[itmPatternPose], scaled_pattern_guess).error_or())
			return err.push_description("failed to configure pattern pose initial guess on compute calibration command");
	}

	// set fixed camera translation axes
	if (translation_camera_fixed) {
		for (int i = 0; i < 3; i++) {
			if (Error err = setNx(calibrate.parameters()[itmFixed][itmLink][itmTranslation][i], (*translation_camera_fixed)[i]).error_or())
				return err.push_description("failed to fix camera translation component: " + std::to_string(i));
		}
	}

	// set fixed camera rotation axes
	if (rotation_camera_fixed) {
		for (int i = 0; i < 3; i++) {
			if (Error err = setNx(calibrate.parameters()[itmFixed][itmLink][itmRotation][i], (*rotation_camera_fixed)[i]).error_or())
				return err.push_description("failed to fix camera rotation component: " + std::to_string(i));
		}
	}

	// set fixed pattern translation axes
	if (translation_pattern_fixed) {
		for (int i = 0; i < 3; i++) {
			if (Error err = setNx(calibrate.parameters()[itmFixed][itmPatternPose][itmTranslation][i], (*translation_pattern_fixed)[i]).error_or())
				return err.push_description("failed to fix pattern translation component: " + std::to_string(i));
		}
	}

	// set fixed pattern rotation axes
	if (rotation_pattern_fixed) {
		for (int i = 0; i < 3; i++) {
			if (Error err = setNx(calibrate.parameters()[itmFixed][itmPatternPose][itmRotation][i], (*rotation_pattern_fixed)[i]).error_or())
				return err.push_description("failed to fix pattern rotation component: " + std::to_string(i));
		}
	}

	// setup (camera in hand / camera fixed)
	Result<void> set_setup_param = setNx(calibrate.parameters()[itmSetup], moving ? valMoving : valFixed);
	if (!set_setup_param) return set_setup_param.error().push_description("failed to configure setup on compute calibration command");

	// name of the target coordinate system
	if (target != "") {
		Result<void> set_target_param = setNx(calibrate.parameters()[itmTarget], target);
		if (!set_target_param) return set_target_param.error().push_description("failed to configure target on compute calibration command");
	}

	// copy robot poses to parameters
	for (size_t i = 0; i < robot_poses.size(); i++) {
		Eigen::Isometry3d scaled_robot_pose = robot_poses[i];
		scaled_robot_pose.translation() *= 1000;
		Result<void> set_transform_param = setNx(calibrate.parameters()[itmTransformations][i], scaled_robot_pose);
		if (!set_transform_param) {
			return set_transform_param.error().push_description("failed to configure transformation on compute calibration command");
		}
	}

	// Optionally copy the parameters for debugging.
	if (parameters_dump_info) *parameters_dump_info = calibrate.parameters().asJson(true);

	Result<void> execute_calibrate = executeNx(calibrate);
	if (!execute_calibrate) {
		if (result_dump_info) *result_dump_info = calibrate.result().asJson(true);
		return execute_calibrate.error();
	}

	// return result (camera pose, pattern pose, iterations, reprojection error)
	Result<Eigen::Isometry3d> camera_pose  = toEigenIsometry(stereo_node[itmLink]);
	if (!camera_pose) camera_pose.error().push_description("failed to retrieve camera pose");
	camera_pose = camera_pose->inverse(); // "Link" is the pose of the world w.r.t. the camera.

	Result<Eigen::Isometry3d> pattern_pose = toEigenIsometry(calibrate.result()[itmPatternPose]);
	if (!pattern_pose) pattern_pose.error().push_description("failed to retrieve patter pose");

	camera_pose->translation()  *= 0.001;
	pattern_pose->translation() *= 0.001;

	// Optionally copy the result for debugging.
	if (result_dump_info) *result_dump_info = calibrate.result().asJson(true);

	Result<int> iterations = getNx<int>(calibrate.result()[itmIterations]);
	if (!iterations) return iterations.error().push_description("failed to retrieve iterations on compute calibration command");

	Result<double> residual = getNx<double>(calibrate.result()[itmResidual]);
	if (!residual) return residual.error().push_description("failed to retrieve residual on compute calibration command");

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
	if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial on set workspace calibration command");

	// scale to [mm]
	Eigen::Isometry3d workspace_mm = workspace;
	workspace_mm.translation() *= 1000;

	Result<void> set_pattern_pose_param = setNx(command.parameters()[itmPatternPose], workspace_mm);
	if (!set_pattern_pose_param) return set_pattern_pose_param.error().push_description("failed to configure pattern pose on set workspace calibration command");

	if (frame_id != "") {
		Result<void> set_target_param = setNx(command.parameters()[itmTarget], frame_id);
		if (!set_target_param) return set_target_param.error().push_description("failed to configure target on set workspace calibration command");
	}

	Eigen::Isometry3d defined_pose_mm = defined_pose;
	defined_pose_mm.translation() *= 1000;

	Result<void> set_defined_pose_param = setNx(command.parameters()[itmDefinedPose], defined_pose_mm);
	if (!set_defined_pose_param) return set_defined_pose_param.error().push_description("failed to configure defined pose on set workspace calibration command");

	Result<void> execute_calibrate_workspace = executeNx(command);
	if (!execute_calibrate_workspace) return execute_calibrate_workspace.error();

	if (store) {
		Result<void> store_workspace_calibration = storeWorkspaceCalibration();
		if (!store_workspace_calibration) return store_workspace_calibration.error().push_description("failed to set workspace calibration");
	}

	return estd::in_place_valid;
}

Result<void> Ensenso::clearWorkspaceCalibration(bool store) {
	// calling CalibrateWorkspace with no PatternPose and DefinedPose clears the workspace.
	NxLibCommand command(cmdCalibrateWorkspace);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial on clear workspace calibration command");

	Result<void> set_target_param = setNx(command.parameters()[itmTarget], "");
	if (!set_target_param) return set_target_param.error().push_description("failed to configure empty target on clear workspace calibration command");

	Result<void> execute_calibrate_workspace = executeNx(command);
	if (!execute_calibrate_workspace) return execute_calibrate_workspace.error();

	// clear target name
	// TODO: Can be removed after settings the target parameter above?
	// TODO: Should test that.
	Result<void> set_stereo_node_target = setNx(stereo_node[itmLink][itmTarget], "");;
	if (!set_stereo_node_target) return set_stereo_node_target.error().push_description("failed to configure empty target");

	if (store) {
		Result<void> store_workspace_calibration = storeWorkspaceCalibration();
		if (!store_workspace_calibration) return store_workspace_calibration.error().push_description("failed to clear workspace calibration");
	}

	return estd::in_place_valid;
}

Result<void> Ensenso::storeWorkspaceCalibration() {
	NxLibCommand command(cmdStoreCalibration);

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) return serial_number.error().push_description("failed to determine stereo camera serial number");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras][0], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to configure stereo camera serial on store workspace calibration command");

	Result<void> set_link_param = setNx(command.parameters()[itmLink], true);
	if (!set_link_param) return set_link_param.error().push_description("failed to configure link on store workspace calibration command");

	return executeNx(command);
}

Result<Eigen::Matrix3d> Ensenso::getMonocularMatrix() const {
	auto matrix = toEigenMatrix(monocular_node.value()[itmCalibration][itmCamera]);
	if (!matrix) return matrix.error().push_description("failed to get monocular camera matrix");

	return *matrix;
}

Result<Eigen::Isometry3d> Ensenso::getMonocularLink() const {
	// NOTE: Ensenso returns the pose of the stereo camera in the monocular frame.
	Result<Eigen::Isometry3d> pose = toEigenIsometry(monocular_node.value()[itmLink]);
	if (!pose) return pose.error().push_description("failed to get monocular link pose");

	// Convert from mm to m.
	pose->translation() *= 0.001;

	return pose;
}

Result<Eigen::Isometry3d> Ensenso::getStereoLink() const {
	// NOTE: Ensenso returns the pose of the base frame in the stereo frame.
	Result<Eigen::Isometry3d> pose = toEigenIsometry(stereo_node[itmLink]);
	if (!pose) return pose.error().push_description("failed to get stereo link pose");

	// Convert from mm to m.
	pose->translation() *= 0.001;

	return pose;
}

Result<Eigen::Isometry3d> Ensenso::getMonoToStereo() const {
	Result<Eigen::Isometry3d> stereo_to_mono = getMonocularLink();
	if (!stereo_to_mono) return stereo_to_mono.error();
	return stereo_to_mono->inverse();
}

Result<Eigen::Isometry3d> Ensenso::getStereoToBase() const {
	Result<Eigen::Isometry3d> base_to_stereo = getStereoLink();
	if (!base_to_stereo) return base_to_stereo.error();
	return base_to_stereo->inverse();
}

Result<Eigen::Isometry3d> Ensenso::getStereoToMono() const {
	return getMonocularLink();
}

Result<Eigen::Isometry3d> Ensenso::getBaseToStereo() const {
	return getStereoLink();
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
		// TODO: use getNx to access values
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

bool Ensenso::isDumpTreeEnabled() {
	if (debug_log_.has_value()) {
		return debug_log_->dump_tree;
	} else {
		return false;
	}
}

void Ensenso::enableDebugLogging(
	std::string const & base_folder,
	DebugLevel const & debug_level,
	int log_file_size,
	bool dump_tree
) {
	namespace fs = boost::filesystem;
	// Set debug log parameters.
	auto log_path = (fs::path(base_folder) / dr::getTimeString()).native();
	auto nxlog_sub_dir = (fs::path(log_path) / "logs").native();
	auto trees_sub_dir = (fs::path(log_path) / "trees").native();
	debug_log_ = DebugLogParams{log_path, nxlog_sub_dir, trees_sub_dir, dump_tree};

	// Create sub directory for nxlogs.
	fs::create_directories(nxlog_sub_dir);

	if (dump_tree) {
		// Create sub directory for camera trees.
		fs::create_directories(trees_sub_dir);
	}

	NxLibItem debug_out = root[itmDebug][itmFileOutput];

	debug_out[itmFolderPath] = nxlog_sub_dir;
	debug_out[itmMaxTotalSize] = log_file_size;
	debug_out[itmEnabled] = true;
	root[itmDebug][itmLevel] = toString(debug_level);
}

Result<void> Ensenso::dumpTree(std::string const & time_stamp) {
	if (!debug_log_.has_value()) {
		return Error{"failed to dump camera tree: debug log is not enabled"};
	}

	Result<std::string> serial_number = serialNumber();
	if (!serial_number) {
		return Error{"failed to dump camera tree: serial number is not found"};
	}

	// Create JSON file.
	std::string tree_file_name = debug_log_->trees_sub_dir + "/" + time_stamp + ".json";
	std::ofstream file(tree_file_name);
	if (!file.is_open()) {
		return Error{fmt::format("failed to save camera tree to '{}'", tree_file_name)};
	}

	// Write entire camera tree as JSON into text file.
	NxLibItem camera = root[itmCameras][itmBySerialNo][*serial_number];
	file << camera.asJson(true);
	file.close();

	// Add the event of camera tree dumping to the nxlog file.
	nxLibWriteDebugMessage(fmt::format("camera tree dumped to '{}'", tree_file_name));

	return estd::in_place_valid;
}

}
