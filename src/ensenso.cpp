#include "ensenso.hpp"
#include "eigen.hpp"
#include "util.hpp"
#include "opencv.hpp"
#include "pcl.hpp"

#include <stdexcept>

namespace dr {

Ensenso::Ensenso(std::string serial, bool connect_overlay) {
	// Initialize nxLib.
	nxLibInitialize();

	if (serial == "") {
		// Try to find a stereo camera.
		boost::optional<NxLibItem> camera = openCameraByType(valStereo);
		if (!camera) throw std::runtime_error("Please connect an Ensenso stereo camera to your computer.");
		ensenso_camera = *camera;
	} else {
		// Open the requested camera.
		boost::optional<NxLibItem> camera = openCameraBySerial(serial);
		if (!camera) throw std::runtime_error("Could not find an Ensenso camera with serial " + serial);
		ensenso_camera = *camera;
	}

	// Get the linked overlay camera.
	if (connect_overlay) overlay_camera = openCameraByLink(serialNumber());
}

Ensenso::~Ensenso() {
	executeNx(NxLibCommand(cmdClose));
	nxLibFinalize();
}

bool Ensenso::loadParameters(std::string const parameters_file) {
	return setNxJsonFromFile(ensenso_camera[itmParameters], parameters_file);
}

bool Ensenso::loadOverlayParameters(std::string const parameters_file) {
	if (!overlay_camera) throw std::runtime_error("No overlay camera found. Can not load overlay parameters.");
	return setNxJsonFromFile(overlay_camera.get()[itmParameters], parameters_file);
}

void Ensenso::loadOverlayParameterSet(std::string const parameters_file) {
	if (!overlay_camera) throw std::runtime_error("No overlay camera found. Can not load overlay parameter set.");
	NxLibCommand command(cmdLoadUEyeParameterSet);
	setNx(command.parameters()[itmFilename], parameters_file);
	executeNx(command);
}

bool Ensenso::trigger(bool stereo, bool overlay) const {
	overlay = overlay && overlay_camera;

	NxLibCommand command(cmdTrigger);
	if (stereo) setNx(command.parameters()[itmCameras][0], serialNumber());
	if (overlay) setNx(command.parameters()[itmCameras][stereo ? 1 : 0], getNx<std::string>(overlay_camera.get()[itmSerialNumber]));
	executeNx(command);

	if (stereo && !getNx<bool>(command.result()[serialNumber()][itmTriggered])) return false;
	if (overlay && !getNx<bool>(command.result()[overlaySerialNumber()][itmTriggered])) return false;
	return true;
}

bool Ensenso::retrieve(bool trigger, unsigned int timeout, bool stereo, bool overlay) const {
	overlay = overlay && overlay_camera;

	NxLibCommand command(trigger ? cmdCapture : cmdRetrieve);
	setNx(command.parameters()[itmTimeout], int(timeout));
	if (stereo) setNx(command.parameters()[itmCameras][0], serialNumber());
	if (overlay) setNx(command.parameters()[itmCameras][stereo ? 1 : 0], getNx<std::string>(overlay_camera.get()[itmSerialNumber]));
	executeNx(command);

	if (stereo && !getNx<bool>(command.result()[serialNumber()][itmRetrieved])) return false;
	if (overlay && !getNx<bool>(command.result()[overlaySerialNumber()][itmRetrieved])) return false;
	return true;
}

bool Ensenso::getPatternPose(Eigen::Isometry3d & pose, int const samples)  {
	discardPatterns();

	for (int i = 0; i < samples; ++i) {
		recordCalibrationPattern();
	}

	// disable FlexView (should not be necessary here, but appears to be necessary for cmdEstimatePatternPose)
	int flex_view = flexView();
	if (flex_view > 0) setFlexView(0);

	// Get the pose of the pattern.
	NxLibCommand command_estimate_pose(cmdEstimatePatternPose);
	executeNx(command_estimate_pose);

	// restore FlexView setting
	if (flex_view > 0) {
		setFlexView(flex_view);
	}

	pose = toEigenIsometry(command_estimate_pose.result()["Patterns"][0][itmPatternPose]);
	pose.translation() *= 0.001;

	// transform back for camera pose
	boost::optional<Eigen::Isometry3d> camera_pose = getCameraPose();
	if (camera_pose) {
		pose = *camera_pose * pose;
	}

	return true;
}

cv::Size Ensenso::getIntensitySize() {
	int width, height;

	try {
		overlay_camera.get()[itmImages][itmRaw].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
	} catch (NxLibException const & e) {
		throw NxError(e);
	}

	return cv::Size(width, height);
}

cv::Size Ensenso::getPointCloudSize() {
	int width, height;
	ensenso_camera[itmImages][itmPointMap].getBinaryDataInfo(&width, &height, 0, 0, 0, 0);
	return cv::Size(width, height);
}

void Ensenso::loadIntensity(cv::Mat & intensity, bool capture) {
	if (capture) this->retrieve(true, 1500, !overlay_camera, !!overlay_camera);

	// Copy to cv::Mat.
	if (overlay_camera) {
		intensity = toCvMat(overlay_camera.get()[itmImages][itmRaw]);
	} else {
		intensity = toCvMat(ensenso_camera[itmImages][itmRaw][itmLeft]);
	}
}

void Ensenso::loadPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi, bool capture) {
	// Optionally capture new data.
	if (capture) this->retrieve();

	setRegionOfInterest(roi);
	std::string serial = serialNumber();

	// Compute disparity.
	{
		NxLibCommand command(cmdComputeDisparityMap);
		setNx(command.parameters()[itmCameras], serial);
		executeNx(command);
	}

	// Compute point cloud.
	{
		NxLibCommand command(cmdComputePointMap);
		setNx(command.parameters()[itmCameras], serial);
		executeNx(command);
	}

	// Convert the binary data to a point cloud.
	cloud = toPointCloud(ensenso_camera[itmImages][itmPointMap]);
}

void Ensenso::loadRegisteredPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi, bool capture) {
	// Optionally capture new data.
	if (capture) this->retrieve();

	setRegionOfInterest(roi);
	std::string serial = serialNumber();

	// Compute disparity.
	{
		NxLibCommand command(cmdComputeDisparityMap);
		setNx(command.parameters()[itmCameras], serial);
		executeNx(command);
	}

	// Render point cloud.
	{
		NxLibCommand command(cmdRenderPointMap);
		setNx(command.parameters()[itmNear], 1); // distance in millimeters to the camera (clip nothing?)
		setNx(command.parameters()[itmCamera], overlaySerialNumber());
		// gives weird (RenderPointMap) results with OpenGL enabled, so disable
		setNx(root[itmParameters][itmRenderPointMap][itmUseOpenGL], false);
		executeNx(command);
	}

	// Convert the binary data to a point cloud.
	cloud = toPointCloud(root[itmImages][itmRenderPointMap]);
}

void Ensenso::setRegionOfInterest(cv::Rect const & roi) {
	if (roi.area() == 0) {
		setNx(ensenso_camera[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest], false);

		if (ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest].exists()) {
			ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest].erase();
		}
	} else {
		setNx(ensenso_camera[itmParameters][itmCapture][itmUseDisparityMapAreaOfInterest],          true);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][0],     roi.tl().x);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmLeftTop][1],     roi.tl().y);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][0], roi.br().x);
		setNx(ensenso_camera[itmParameters][itmDisparityMap][itmAreaOfInterest][itmRightBottom][1], roi.br().y);
	}
}

void Ensenso::discardPatterns() {
	executeNx(NxLibCommand(cmdDiscardPatterns));
}

void Ensenso::recordCalibrationPattern() {
	// disable FlexView
	int flex_view = flexView();
	if (flex_view > 0) setFlexView(0);

	// Capture image with front-light.
	setNx(ensenso_camera[itmParameters][itmCapture][itmProjector], false);
	setNx(ensenso_camera[itmParameters][itmCapture][itmFrontLight], true);

	retrieve(true, 1500, true, false);

	setNx(ensenso_camera[itmParameters][itmCapture][itmFrontLight], false);
	setNx(ensenso_camera[itmParameters][itmCapture][itmProjector], true);

	// Find the pattern.
	NxLibCommand command_collect_pattern(cmdCollectPattern);
	setNx(command_collect_pattern.parameters()[itmCameras], serialNumber());
	setNx(command_collect_pattern.parameters()[itmDecodeData], true);

	executeNx(command_collect_pattern);

	// restore FlexView setting
	if (flex_view > 0) {
		setFlexView(flex_view);
	}
}

Ensenso::CalibrationResult Ensenso::computeCalibration(
	std::vector<Eigen::Isometry3d> const & robot_poses,
	bool moving,
	boost::optional<Eigen::Isometry3d> const & camera_guess,
	boost::optional<Eigen::Isometry3d> const & pattern_guess,
	std::string const & target
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

	// execute calibration command
	executeNx(calibrate);

	// return result (camera pose, pattern pose, iterations, reprojection error)
	Eigen::Isometry3d camera_pose  = toEigenIsometry(ensenso_camera[itmLink]).inverse(); // "Link" is inverted
	Eigen::Isometry3d pattern_pose = toEigenIsometry(calibrate.result()[itmPatternPose]);
	camera_pose.translation()  *= 0.001;
	pattern_pose.translation() *= 0.001;

	return Ensenso::CalibrationResult{
		camera_pose,
		pattern_pose,
		getNx<int>(calibrate.result()[itmIterations]),
		getNx<double>(calibrate.result()[itmReprojectionError])
	};
}

boost::optional<std::string> Ensenso::getFrame() {
	// Make sure the relevant nxLibItem exists and is non-empty, then return it.
	NxLibItem item = ensenso_camera[itmLink][itmTarget];
	if (!item.exists()) return boost::none;
	std::string frame = getNx<std::string>(item);
	if (frame.empty()) return boost::none;
	return frame;
}

boost::optional<Eigen::Isometry3d> Ensenso::getCameraPose() {
	// Check if the camera is calibrated.
	if (!getFrame()) return boost::none;

	// convert from mm to m
	Eigen::Isometry3d pose = toEigenIsometry(ensenso_camera[itmLink]);
	pose.translation() *= 0.001;

	return pose;
}

void Ensenso::clearWorkspace() {
	// Check if the camera is calibrated.
	if (!getFrame()) return;

	// calling CalibrateWorkspace with no PatternPose and DefinedPose clears the workspace.
	NxLibCommand command(cmdCalibrateWorkspace);
	setNx(command.parameters()[itmCameras][0], serialNumber());
	setNx(command.parameters()[itmTarget], "");
	executeNx(command);

	// clear target name
	// TODO: Can be removed after settings the target parameter above?
	// TODO: Should test that.
	setNx(ensenso_camera[itmLink][itmTarget], "");
}

void Ensenso::setWorkspace(Eigen::Isometry3d const & workspace, std::string const & frame_id, Eigen::Isometry3d const & defined_pose) {
	// calling CalibrateWorkspace with no PatternPose and DefinedPose clears the workspace.
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
}

void Ensenso::storeCalibration() {
	NxLibCommand command(cmdStoreCalibration);
	setNx(command.parameters()[itmCameras][0], serialNumber());
	executeNx(command);
}

}
