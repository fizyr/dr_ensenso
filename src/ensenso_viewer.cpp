#include "ensenso.hpp"

#include <dr_camera/camera_viewer.hpp>

namespace dr {

class EnsensoViewer : public CameraViewer {
public:
	EnsensoViewer() :
		CameraViewer(true, 1, 0),
		ensenso(new dr::Ensenso()) {
	}

protected:
	virtual std::shared_ptr<IntensityCamera> getIntensityCamera(uint8_t id) const {
		(void) id;
		return ensenso;
	}

	virtual std::shared_ptr<DepthCamera> getDepthCamera(uint8_t id) const {
		(void) id;
		return nullptr;
	}

	virtual std::shared_ptr<PointCloudCamera> getPointCloudCamera(uint8_t id) const {
		(void) id;
		return ensenso;
	}

	/// The depthsense camera device
	std::shared_ptr<dr::Ensenso> ensenso;
};

}

int main(int argc, char * argv[]) {
	ros::init(argc, argv, "ensenso_viewer");
	dr::EnsensoViewer viewer;
	viewer.spin();
}
