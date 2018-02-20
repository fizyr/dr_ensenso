#include "util.hpp"

#include <fstream>
#include <sstream>

namespace dr {

std::optional<NxLibItem> findCameraBySerial(std::string const & serial) {
	NxLibItem camera = NxLibItem{}[itmCameras][itmBySerialNo][serial];
	if (!camera.exists()) return {};
	return camera;
}

std::optional<NxLibItem> findCameraByEepromId(int eeprom_id) {
	NxLibItem camera = NxLibItem{}[itmCameras][itmByEepromId][eeprom_id];
	if (!camera.exists()) return {};
	return camera;
}

std::optional<NxLibItem> findCameraByLink(std::string const & serial) {
	NxLibItem cameras = NxLibItem{}[itmCameras][itmBySerialNo];
	for (int i = 0; i < cameras.count(); ++i) {
		if (getNx<std::string>(cameras[i][itmLink][itmTarget]) == serial) return cameras[i];
	}
	return {};
}

std::optional<NxLibItem> findCameraByType(std::string const & type) {
	NxLibItem cameras = NxLibItem{}[itmCameras][itmBySerialNo];
	for (int i = 0; i < cameras.count(); ++i) {
		if (getNx<std::string>(cameras[i][itmType]) == type) return cameras[i];
	}
	return {};
}

namespace {
	/// Open an optional camera, or return nothing.
	std::optional<NxLibItem> openCamera(std::optional<NxLibItem> camera) {
	if (!camera) return {};

		NxLibCommand command(cmdOpen);
		setNx(command.parameters()[itmCameras], getNx<std::string>((*camera)[itmSerialNumber]));
		executeNx(command);

		return camera;
	}
}

std::optional<NxLibItem> openCameraBySerial(std::string const & serial) {
	return openCamera(findCameraBySerial(serial));
}

std::optional<NxLibItem> openCameraByEepromId(int eeprom_id) {
	return openCamera(findCameraByEepromId(eeprom_id));
}

std::optional<NxLibItem> openCameraByLink(std::string const & serial) {
	return openCamera(findCameraByLink(serial));
}

std::optional<NxLibItem> openCameraByType(std::string const & type) {
	return openCamera(findCameraByType(type));
}

void executeNx(NxLibCommand const & command, std::string const & what) {
	int error = 0;
	command.execute(&error);
	if (error) throwCommandError(error, what);
}

std::int64_t getNxBinaryTimestamp(NxLibItem const & item, std::string const & what) {
	int error = 0;
	double timestamp = 0;
	item.getBinaryDataInfo(&error, nullptr, nullptr, nullptr, nullptr, nullptr, &timestamp);
	if (error) throw NxError(item, error, what);
	return (timestamp - 11644473600.0) * 1e6; // Correct for epoch and turn into microseconds.
}

void setNxJson(NxLibItem const & item, std::string const & json, std::string const & what) {
	int error = 0;
	item.setJson(&error, json, true);
	if (error) throw NxError(item, error, what);
}

bool setNxJsonFromFile(NxLibItem const & item, std::string const & filename, std::string const & what) {
	std::ifstream file;
	file.open(filename);

	if (!file.good()) {
		return false;
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	std::stringstream buffer;
	buffer << file.rdbuf();
	setNxJson(item, buffer.str(), what);

	return true;
}

std::string getNxJson(NxLibItem const & item, std::string const & what) {
	int error = 0;
	std::string result = item.asJson(&error, true);
	if (error) throw NxError(item, error, what);
	return result;
}

void writeNxJsonToFile(NxLibItem const & item, std::string const & filename, std::string const & what) {
	std::ofstream file;
	file.exceptions(std::ios::failbit | std::ios::badbit);
	file.open(filename);
	file << getNxJson(item, what);
}

}
