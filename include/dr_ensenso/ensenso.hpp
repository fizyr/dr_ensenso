#pragma once

#include <Eigen/Eigen>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <estd/utility/move_marker.hpp>

#include <ensenso/nxLib.h>
#include <optional>

namespace dr {

using LogFunction = std::function<void (std::string)>;

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

class NxLibInitGuard {
private:
	bool moved_ = false;

public:
	NxLibInitGuard();
	NxLibInitGuard(NxLibInitGuard const &) = delete;
	NxLibInitGuard(NxLibInitGuard &&);
	NxLibInitGuard& operator=(NxLibInitGuard const &) = delete;
	NxLibInitGuard& operator=(NxLibInitGuard &&);
	~NxLibInitGuard();
};

using NxLibInitToken = std::shared_ptr<NxLibInitGuard>;

inline NxLibInitToken initNxLib() {
	return std::make_shared<NxLibInitGuard>();
}

class Ensenso {
public:
	/// Ensenso calibration result (camera pose, pattern pose, iterations needed, residual error).
	using CalibrationResult = std::tuple<Eigen::Isometry3d, Eigen::Isometry3d, int, double>;

	/// Camera capture parameters, parsed from the cameras.
	struct CaptureParams {
		std::size_t stereo_width;                    // Width of image from stereo camera.
		std::size_t stereo_height;                   // Height of image from stereo camera.
		std::optional<std::size_t> monocular_width;  // Width of image from monocular camera.
		std::optional<std::size_t> monocular_height; // Height of image from monocular camera.
	};

protected:
	/// The root EnsensoSDK node.
	NxLibItem root;

	/// The Ensenso camera node.
	NxLibItem stereo_node;

	/// The attached monocular camera node.
	std::optional<NxLibItem> monocular_node;

	/// Initializtion token to allow refcounted initialization of NxLib.
	NxLibInitToken init_token_;

	/// Log function to use for verbose logging.
	LogFunction logger_;

public:
	constexpr static bool needMonocular(ImageType image) {
		switch (image) {
			case ImageType::monocular_raw:
			case ImageType::monocular_rectified:
			case ImageType::monocular_overlay:
				return true;
			case ImageType::stereo_raw_left:
			case ImageType::stereo_raw_right:
			case ImageType::stereo_rectified_left:
			case ImageType::stereo_rectified_right:
			case ImageType::disparity:
				return false;
		}
		return false;
	}

	/// Connect to an ensenso camera.
	Ensenso(std::string serial = "", bool connect_monocular = true, LogFunction log = nullptr, NxLibInitToken init_token = initNxLib());

	/// Explicitly opt-in to default move semantics.
	Ensenso(Ensenso &&)       = default;
	Ensenso(Ensenso const &)  = delete;
	Ensenso & operator=(Ensenso      &&) = default;
	Ensenso & operator=(Ensenso const &) = delete;

	/// Destructor.
	~Ensenso();

	/// Get the native nxLibItem for the stereo camera.
	NxLibItem native() const {
		return stereo_node;
	}

	/// Get the native nxLibItem for the monocular camera (if any).
	std::optional<NxLibItem> nativeMonocular() const {
		return monocular_node;
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
	bool loadParameters(std::string const parameters_file, bool entire_tree = false);

	/// Loads the monocular camera parameters from a JSON file. Returns false if file was not found.
	bool loadMonocularParameters(std::string const parameters_file, bool entire_tree = false);

	/// Loads the monocular camera uEye parameters from a INI file. Returns false if file was not found.
	void loadMonocularUeyeParameters(std::string const parameters_file);

	/// Check if the camera has a FlexView parameter.
	bool hasFlexView() const;

	/// Returns the current FlexView value. If disabled, returns -1.
	int flexView() const;

	/// Sets the Ensenso camera FlexView value.
	void setFlexView(int value);

	/// Check if the camera has a front light.
	bool hasFrontLight() const;

	/// Get the front light setting (on or off).
	std::optional<bool> frontLight();

	/// Sets the front light on or off.
	void setFrontLight(bool state);

	/// Get the projector setting (on or off).
	bool projector();

	/// Sets the projector on or off.
	void setProjector(bool state);

	/// Set the region of interest for the disparity map (and thereby depth / point cloud).
	/**
	 * An empty (default constructed) rect clears the ROI.
	 */
	void setDisparityRegionOfInterest(cv::Rect const & roi);

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
	/**
	 * \param stereo If true, rectify data from the stereo camera.
	 * \param monocular If true, rectify data from the monocular camera.
	 */
	void rectifyImages(bool stereo, bool monocular);

	/// Compute the disparity.
	void computeDisparity();

	/// Compute the point cloud.
	void computePointCloud();

	/// Register the point cloud to the RGB frame.
	void registerPointCloud();

	/// Load an image from the camera.
	/**
	 * The image must have been captured, retrieved and/or processed before it can be loaded.
	 */
	cv::Mat loadImage(ImageType type);

	/// Load an image from the camera.
	/**
	 * The image must have been captured, retrieved and/or processed before it can be loaded.
	 */
	void loadImage(
		ImageType type,
		std::uint8_t * pointer,
		std::size_t width,
		std::size_t height,
		int cv_type
	);


	/// Load the point cloud from the camera.
	/**
	 * The point cloud must have been computed before it can be loaded.
	 *
	 * \param roi The region of interest.
	 */
	pcl::PointCloud<pcl::PointXYZ> loadPointCloud();

	/// Loads the pointcloud registered to the monocular camera.
	/**
	 * The point cloud must have been computed and registered before it can be loaded.
	 *
	 * \param roi The region of interest.
	 */
	pcl::PointCloud<pcl::PointXYZ> loadRegisteredPointCloud();

	/// Discards all stored calibration patterns.
	void discardCalibrationPatterns();

	/// Records a calibration pattern.
	void recordCalibrationPattern(
		std::string * parameters_dump_info = nullptr, ///< If provided, copies the parameters to this string as json.
		std::string * result_dump_info = nullptr      ///< If provided, copies the result to this string as json.
	);

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
	std::optional<Eigen::Isometry3d> getWorkspaceCalibration();

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

	/// Performs calibration using previously recorded calibration results and the corresponding robot poses.
	CalibrationResult computeCalibration(
		std::vector<Eigen::Isometry3d> const & robot_poses,          ///< Vector of robot poses corresponding to the stored calibration patterns.
		bool camera_moving,                                          ///< If true, the camera is expected to be in hand. Otherwise the camera is expected to be fixed.
		std::optional<Eigen::Isometry3d> const & camera_guess = {},  ///< Initial guess for the camera relative to the hand (camera in hand) or camera relative to robot base (camera fixed). Not necessary, but speeds up calibration.
		std::optional<Eigen::Isometry3d> const & pattern_guess = {}, ///< Initial guess for the pattern relative to the hand (camera in hand) or pattern relative to robot base (camera fixed). Not necessary, but speeds up calibration.
		std::string const & target = "",                             ///< Target frame to calibrate to. Default is "Hand" for camera in hand and "Workspace" for fixed camera.
		std::string * parameters_dump_info = nullptr,                ///< If provided, copies the parameters to this string as json.
		std::string * result_dump_info = nullptr                     ///< If provided, copies the result to this string as json.
	);

	/// Returns the calibration between ueye and ensenso.
	Eigen::Isometry3d getMonocularLink() const;

	/// Gets capture parameters.
	CaptureParams getCaptureParameters();

protected:

	/// Log a message using the log function registered at contruction time.
	void log(std::string message) const {
		if (logger_) logger_(std::move(message));
	}
};

}
