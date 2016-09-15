#pragma once

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

	/// The attached monocular camera node.
	boost::optional<NxLibItem> monocular_camera;

public:
	/// Ensenso calibration result (camera pose, pattern pose, iterations needed, reprojection error).
	using CalibrationResult = std::tuple<Eigen::Isometry3d, Eigen::Isometry3d, int, double>;

	/// Connect to an ensenso camera.
	Ensenso(std::string serial = "", bool connect_monocular = true);

	/// Destructor.
	~Ensenso();

	/// Get the native nxLibItem for the stereo camera.
	NxLibItem native() const {
		return ensenso_camera;
	}

	/// Get the native nxLibItem for the monocular camera (if any).
	boost::optional<NxLibItem> nativeMonocular() const {
		return monocular_camera;
	}

	/// Returns whether the Ensenso has a monocular camera.
	bool hasMonocular() const {
		return !!nativeMonocular();
	}

	/// Get the serial number of the stereo camera.
	std::string serialNumber() const;

	/// Get the serial number of the monocular camera or an empty string if there is no monocular camera.
	std::string monocularSerialNumber() const;

	/// Loads the camera parameters from a JSON file.
	bool loadParameters(std::string const parameters_file);

	/// Loads the monocular camera parameters from a JSON file. Returns false if file was not found.
	bool loadMonocularParameters(std::string const parameters_file);

	/// Loads the monocular camera uEye parameters from a INI file. Returns false if file was not found.
	void loadMonocularUeyeParameters(std::string const parameters_file);

	/// Returns the current FlexView value. If disabled, returns -1.
	int flexView() const;

	/// Sets the Ensenso camera FlexView value.
	void setFlexView(int value);

	/// Sets the front light on or off.
	void setFrontLight(bool state);

	/// Sets the projector on or off.
	void setProjector(bool state);

	/// Trigger data acquisition on the camera.
	/**
	 * \param stereo If true, capture data from the stereo camera.
	 * \param monocular If true, capture data from the monocular camera.
	 */
	bool trigger(bool stereo = true, bool monocular=true) const;

	/// Retrieve new data from the camera without sending a software trigger.
	/**
	 * \param timeout A timeout in milliseconds.
	 * \param stereo If true, capture data from the stereo camera.
	 * \param monocular If true, capture data from the monocular camera.
	 */
	bool retrieve(bool trigger = true, unsigned int timeout = 1500, bool stereo = true, bool monocular=true) const;

	/// Rectifies the images.
	void rectifyImages();

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

	/// Loads the pointcloud registered to the monocular camera.
	/**
	 * \param cloud the resulting pointcloud.
	 * \param roi The region of interest.
	 * \param capture If true, capture a new image before loading the point cloud.
	 */
	void loadRegisteredPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud, cv::Rect roi = cv::Rect(), bool capture = true);

	/// Discards all stored calibration patterns.
	void discardCalibrationPatterns();

	/// Records a calibration pattern.
	void recordCalibrationPattern();

	/// Detect the calibration pattern and estimate the pose of the pattern.
	/**
	 * This function will record a number of images and detect the calibration pattern on each image.
	 *
	 * The returned pose can be relative to the calibrated frame, or relative to the left stereo lens (default).
	 *
	 * \return The estimated pose of the pattern.
	 */
	Eigen::Isometry3d detectCalibrationPattern(
		int const samples,               ///< The number of samples to record.
		bool ignore_calibration = false  ///< If true, give the pose relative to the left stereo lens.
	);

	/// Get the frame the camera is calibrated to, or an empty string.
	std::string getWorkspaceCalibrationFrame();

	/// Get the active workspace calibration.
	/**
	 * \return The pose of the camera in the calibrated frame, if the camera is calibrated. Otherwise an empty optional.
	 */
	boost::optional<Eigen::Isometry3d> getWorkspaceCalibration();

	/// Performs calibration using previously recorded calibration results and the corresponding robot poses.
	CalibrationResult computeCalibration(
		std::vector<Eigen::Isometry3d> const & robot_poses,            ///< Vector of robot poses corresponding to the stored calibration patterns.
		bool camera_moving,                                            ///< If true, the camera is expected to be in hand. Otherwise the camera is expected to be fixed.
		boost::optional<Eigen::Isometry3d> const & camera_guess = {},  ///< Initial guess for the camera relative to the hand (camera in hand) or camera relative to robot base (camera fixed). Not necessary, but speeds up calibration.
		boost::optional<Eigen::Isometry3d> const & pattern_guess = {}, ///< Initial guess for the pattern relative to the hand (camera in hand) or pattern relative to robot base (camera fixed). Not necessary, but speeds up calibration.
		std::string const & target = ""                                ///< Target frame to calibrate to. Default is "Hand" for camera in hand and "Workspace" for fixed camera.
	);

	/// Sets the active workspace calibration.
	void setWorkspaceCalibration(
		Eigen::Isometry3d const & workspace,
		std::string const & frame_id = "Workspace",
		Eigen::Isometry3d const & defined_pose = Eigen::Isometry3d::Identity(),
		bool store = false
	);

	/// Clears the active workspace calibration.
	void clearWorkspaceCalibration(bool store = false);

	/// Stores the active workspace caliration on the EEPROM of the camera.
	void storeWorkspaceCalibration();

protected:
	/// Set the region of interest for the disparity map (and thereby depth / point cloud).
	void setRegionOfInterest(cv::Rect const & roi);

};

}
