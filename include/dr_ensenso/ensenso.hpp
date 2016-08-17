#pragma once
#include "util.hpp"

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ensenso/nxLib.h>
#include <boost/optional.hpp>

namespace dr {

class Ensenso {
protected:
	/// The root EnsensoSDK node.
	NxLibItem root;

	/// The Ensenso camera node.
	NxLibItem ensenso_camera;

	/// The overlay camera node.
	boost::optional<NxLibItem> overlay_camera;

public:
	/// Ensenso calibration result (camera pose, pattern pose, iterations needed, reprojection error).
	using CalibrationResult = std::tuple<Eigen::Isometry3d, Eigen::Isometry3d, int, double>;

	/// Connect to an ensenso camera.
	Ensenso(std::string serial = "", bool connect_overlay = true);

	/// Destructor.
	~Ensenso();

	/// Get the native nxLibItem for the stereo camera.
	NxLibItem native() const {
		return ensenso_camera;
	}

	/// Get the native nxLibItem for the overlay camera (if any).
	boost::optional<NxLibItem> nativeOverlay() const {
		return overlay_camera;
	}

	/// Returns whether the Ensenso has an overlay camera.
	bool hasOverlay() const {
		return nativeOverlay() != boost::none;
	}

	/// Get the serial number of the stereo camera.
	std::string serialNumber() const {
		return getNx<std::string>(ensenso_camera[itmSerialNumber]);
	}

	/// Get the serial number of the overlay camera or an empty string if there is no overlay camera.
	std::string overlaySerialNumber() const {
		return overlay_camera ? getNx<std::string>(overlay_camera.get()[itmSerialNumber]) : "";
	}

	/// Loads the camera parameters from a JSON file.
	bool loadParameters(std::string const parameters_file);

	/// Loads the overlay camera parameters from a JSON file. Returns false if file was not found.
	bool loadOverlayParameters(std::string const parameters_file);

	/// Loads the overlay camera uEye parameters from a INI file. Returns false if file was not found.
	void loadOverlayParameterSet(std::string const parameters_file);

	/// Trigger data acquisition on the camera.
	/**
	 * \param stereo If true, capture data from the stereo camera.
	 * \param overlay If true, capture data from the overlay camera.
	 */
	bool trigger(bool stereo = true, bool overlay=true) const;

	/// Retrieve new data from the camera without sending a software trigger.
	/**
	 * \param timeout A timeout in milliseconds.
	 * \param stereo If true, capture data from the stereo camera.
	 * \param overlay If true, capture data from the overlay camera.
	 */
	bool retrieve(bool trigger = true, unsigned int timeout = 1500, bool stereo = true, bool overlay=true) const;

	/// Returns the pose of the calibration plate with respect to the camera.
	bool calibrate(int const num_patterns, Eigen::Isometry3d & pose);

	/// Returns the size of the intensity images.
	cv::Size getIntensitySize();

	/// Returns the size of the depth images.
	cv::Size getPointCloudSize();

	/// Loads the intensity image to intensity.
	/**
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadIntensity(cv::Mat & intensity, bool capture);

	/// Loads the intensity image to intensity.
	void loadIntensity(cv::Mat & intensity) {
		loadIntensity(intensity, true);
	}

	/// Get the intensity image.
	/**
	 * \return The intensity image.
	 */
	cv::Mat getIntensity() {
		cv::Mat intensity;
		loadIntensity(intensity);
		return intensity;
	}

	/// Loads the pointcloud from depth in the region of interest.
	/**
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi, bool capture);

	/// Loads the pointcloud from depth in the region of interest.
	/**
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 */
	void loadPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi = cv::Rect()) {
		return loadPointCloud(cloud, roi, true);
	}

	/// Get a pointlcoud from the camera.
	/**
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	pcl::PointCloud<pcl::PointXYZ> getPointCloud(cv::Rect roi = cv::Rect(), bool capture = true) {
		pcl::PointCloud<pcl::PointXYZ> result;
		loadPointCloud(result, roi, capture);
		return result;
	}

	/// Loads the pointcloud registered to the overlay camera.
	/**
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadRegisteredPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi = cv::Rect(), bool capture = true);

	/// Discards all stored calibration patterns.
	void discardPatterns();

	/// Records a calibration pattern.
	void recordCalibrationPattern();

	/// Performs calibration using previously recorded calibration results and the corresponding robot poses.
	CalibrationResult computeCalibration(
		std::vector<Eigen::Isometry3d> const & robot_poses,            ///< Vector of robot poses corresponding to the stored calibration patterns.
		bool moving,                                                   ///< If true, the camera is expected to be in hand. Otherwise the camera is expected to be fixed.
		boost::optional<Eigen::Isometry3d> const & camera_guess = {},  ///< Initial guess for the camera relative to the hand (camera in hand) or camera relative to robot base (camera fixed). Not necessary, but speeds up calibration.
		boost::optional<Eigen::Isometry3d> const & pattern_guess = {}, ///< Initial guess for the pattern relative to the hand (camera in hand) or pattern relative to robot base (camera fixed). Not necessary, but speeds up calibration.
		std::string const & target = ""                                ///< Target frame to calibrate to. Default is "Hand" for camera in hand and "Workspace" for fixed camera.
	);

	/// Get the frame the camera is calibrated to, if any.
	boost::optional<std::string> getFrame();

	/// Get the pose of the camera in the calibrated frame, if the camera is calibrated.
	boost::optional<Eigen::Isometry3d> getCameraPose();

	/// Clears the Workspace, if it exists.
	void clearWorkspace();

	/// Returns the current FlexView value. If disabled, returns -1.
	int flexView() {
		try {
			// in case FlexView = false, getting the int value gives an error
			return getNx<int>(ensenso_camera[itmParameters][itmCapture][itmFlexView]);
		} catch (NxError const & e) {
			return -1;
		}
	}

	/// Sets the Ensenso camera FlexView value.
	void setFlexView(int value) {
		setNx(ensenso_camera[itmParameters][itmCapture][itmFlexView], value);
	}

	/// Sets the Workspace calibration link.
	void setWorkspace(Eigen::Isometry3d const & workspace);

	/// Stores the caliration on the EEPROM of the camera.
	void storeCalibration();

protected:
	/// Set the region of interest for the disparity map (and thereby depth / point cloud).
	void setRegionOfInterest(cv::Rect const & roi);

};

}
