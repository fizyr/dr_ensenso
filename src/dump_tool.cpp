#include "ensenso.hpp"
#include "util.hpp"
#include "dump.hpp"

#include <dr_util/timestamp.hpp>

#include <boost/date_time/gregorian/gregorian.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/time_facet.hpp>
#include <boost/filesystem/operations.hpp>

#include <sstream>
#include <csignal>
#include <string>

namespace dr {

/// Tool for dumping live data from an Ensenso camera to files.
class EnsensoDumpTool {
	/// Ensenso camera wrapper.
	Ensenso ensenso_;

	/// Raw ensenso nxLib node.
	NxLibItem camera_;

	/// Optional raw overlay camera nxLib node.
	boost::optional<NxLibItem> overlay_;

	/// Flag to asynchronously stop the dump tool if it is running.
	volatile bool stop_;

public:
	/// The capture timeout in milliseconds.
	int timeout = 1500;

	/// The directory to save the output files to.
	std::string output_directory;

	struct {
		bool stereo_raw        = true;
		bool stereo_rectified  = true;
		bool disparity         = true;
		bool point_cloud       = true;
		bool overlay_raw       = true;
		bool overlay_rectified = true;
	} dump;

	/// Check the camera wrapper.
	Ensenso const & camera() const { return ensenso_; }

	/// Construct an ensenso dump tool.
	EnsensoDumpTool() {
		camera_  = ensenso_.native();
		overlay_ = ensenso_.nativeOverlay();

		output_directory = getTimeString();
	}

	/// Destructor.
	~EnsensoDumpTool() {
		std::cerr << "Closing the camera. This may take some time.\n";
	}

	/// Load camera parameters for the main stereo camera.
	void loadMainParameters(std::string const & filename) {
		std::cerr << "Loading camera parameters from " << filename << "\n";
		ensenso_.loadParameters(filename);
	}

	/// Load camera parameters for the linked overlay camera.
	void loadOverlayParameters(std::string const & filename) {
		std::cerr << "Loading overlay camera parameters from " << filename << "\n";
		ensenso_.loadOverlayParameters(filename);
	}

	/// Set the connected cameras hardware or software triggered.
	void hardwareTriggered(bool enabled) {
		setNx(camera_[itmParameters][itmCapture][itmTriggerMode], enabled ? valFallingEdge : valSoftware);
		if (overlay_) {
			setNx((*overlay_)[itmParameters][itmCapture][itmTriggerMode], enabled ? valRisingEdge : valSoftware);
		}
	}

	/// Dump parameters.
	void dumpParameters() {
		boost::filesystem::create_directories({output_directory});
		dr::dumpParameters(camera_[itmParameters],  output_directory + "/", "_" + ensenso_.serialNumber() + "_parameters");
		dr::dumpParameters(camera_[itmCalibration], output_directory + "/", "_" + ensenso_.serialNumber() + "_calibration");
		dr::dumpParameters(camera_[itmLink],        output_directory + "/", "_" + ensenso_.serialNumber() + "_link");
		dr::dumpParametersYaml(camera_[itmCalibration], "Left", output_directory + "/", "_" + ensenso_.serialNumber());
		dr::dumpParametersYaml(camera_[itmCalibration], "Right", output_directory + "/", "_" + ensenso_.serialNumber());
		if (overlay_) {
			dr::dumpParameters(overlay_.get()[itmParameters],  output_directory + "/", "_" + ensenso_.overlaySerialNumber() + "_parameters");
			dr::dumpParameters(overlay_.get()[itmCalibration], output_directory + "/", "_" + ensenso_.overlaySerialNumber() + "_calibration");
			dr::dumpParameters(overlay_.get()[itmLink],        output_directory + "/", "_" + ensenso_.overlaySerialNumber() + "_link");
		}
	}

	bool recordData() {
		ensenso_.trigger();
		try {
			if (!ensenso_.retrieve(false, timeout)) return false;
		} catch (NxCommandError const & e) {
			// Ignore timeouts, throw the rest of the errors.
			if (e.error_symbol() == errCaptureTimeout) {
				return false;
			}
			throw;
		}

		return true;
	}

	/// Do a single trigger, retrieve, process, dump step.
	void step() {
		if (!recordData()) return;

		// Compute disparity if needed.
		if (dump.disparity || dump.point_cloud) {
			NxLibCommand command(cmdComputeDisparityMap);
			setNx(command.parameters()[itmCameras], ensenso_.serialNumber());
			executeNx(command);
		}

		// Compute point cloud if needed.
		if (dump.point_cloud) {
			NxLibCommand command(cmdComputePointMap);
			setNx(command.parameters()[itmCameras], ensenso_.serialNumber());
			executeNx(command);
		}

		// Also get rectified overlay image.
		if (overlay_ && dump.overlay_rectified) {
			NxLibCommand command{cmdRectifyImages};
			setNx(command.parameters()[itmCameras], ensenso_.overlaySerialNumber());
			executeNx(command);
		}

		std::cerr << "Data retrieved.\n";
		dumpData();
	}

	/// Continually run step() until stopped.
	void run() {
		stop_ = false;
		while (!stop_) step();
	}

	/// Stop the dump tool if it is currently running.
	void stop() {
		stop_ = true;
	}

protected:
	/// Dump all data
	void dumpData() {
		std::string sample_directory = output_directory + "/" + getTimeString();
		boost::filesystem::create_directories({sample_directory});

		dumpCameraImages(
			camera_,
			sample_directory + "/",
			"",
			default_time_format,
			dump.stereo_raw,
			dump.stereo_rectified,
			dump.disparity,
			dump.point_cloud
		);

		if (overlay_) dumpCameraImages(
			*overlay_,
			sample_directory + "/",
			"",
			default_time_format,
			dump.overlay_raw,
			dump.overlay_rectified,
			false,
			false
		);
	}
};

}

namespace {
	dr::EnsensoDumpTool * dump_tool = nullptr;

	extern "C" void signal_handler(int) {
		if (dump_tool) dump_tool->stop();
	}
}

int main(int argc, char * * argv) {
	std::cerr << "Usage: " << argv[0] << " <timeout (ms)> <optional path to camera parameters> <optional path to overlay camera parameters>" << std::endl;
	// Create and configure Ensenso
	std::cerr << "Initializing camera. This may take some time.\n";

	dr::EnsensoDumpTool dump_tool;
	::dump_tool = &dump_tool;
	std::signal(SIGINT, signal_handler);
	std::cerr << "Camera initialized. Setting parameters.\n";

	if (argc > 1) dump_tool.timeout = std::atoi(argv[1]);
	if (argc > 2) dump_tool.loadMainParameters(argv[2]);
	if (argc > 3) dump_tool.loadOverlayParameters(argv[3]);
	dump_tool.hardwareTriggered(true);
	dump_tool.dumpParameters();

	std::cerr << "Starting camera capture.\n";
	dump_tool.run();
	std::cerr << "Stopped image capture.\n";
	std::signal(SIGINT, SIG_DFL);
}
