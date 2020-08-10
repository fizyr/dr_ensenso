#include "util.hpp"

#include <fstream>
#include <sstream>

#include <fmt/format.h>

namespace dr {

std::optional<NxLibItem> findCameraBySerial(std::string const & serial) {
	NxLibItem camera = NxLibItem{}[itmCameras][serial];
	if (!camera.exists()) {
		return {};
	}
	return camera;
}

std::optional<NxLibItem> findCameraByEepromId(int eeprom_id) {
	NxLibItem camera = NxLibItem{}[itmCameras][itmByEepromId][eeprom_id];
	if (!camera.exists()) return {};
	return camera;
}

std::optional<NxLibItem> findCameraByLink(std::string const & linked_to, LogFunction logger) {
	if (logger) logger(fmt::format("Looking for camera linked to target {}.", linked_to));
	NxLibItem cameras = NxLibItem{}[itmCameras];
	for (int i = 0; i < cameras.count(); ++i) {
		Result<std::string> serial = getNx<std::string>(cameras[i][itmSerialNumber]);
		if (!serial) continue;

		NxLibItem camera = cameras[*serial];

		if (!camera[itmLink][itmTarget].exists()) {
			if (logger) logger(fmt::format("Camera with serial {} is not linked to anything."));
			continue;
		}
		Result<std::string> target = getNx<std::string>(camera[itmLink][itmTarget]);
		if (!target) continue;

		if (target == linked_to) {
			if (logger) logger(fmt::format("Camera with serial {} is linked our target: {}", serial, target));
			return camera;
		} else {
			if (logger) logger(fmt::format("Camera with serial {} is not linked to our target: {}", serial, target));
		}
	}
	if (logger) logger(fmt::format("No camera linked to our target {}  found.", linked_to));
	return {};
}

std::optional<NxLibItem> findCameraByType(std::string const & wanted_type, LogFunction logger) {
	if (logger) logger(fmt::format("Looking for camera of type {}.", wanted_type));
	NxLibItem cameras = NxLibItem{}[itmCameras];
	for (int i = 0; i < cameras.count(); ++i) {
		Result<std::string> serial = getNx<std::string>(cameras[i][itmSerialNumber]);
		if (!serial) continue;

		NxLibItem camera = cameras[*serial];

		Result<std::string> type = getNx<std::string>(camera[itmType]);
		if (!type) continue;

		if (type == wanted_type) {
			if (logger) logger(fmt::format("Camera with serial {} is the wanted type: {}", serial, type));
			return camera;
		} else {
			if (logger) logger(fmt::format("Camera with serial {} is not the wanted type: {}", serial, type));
		}
	}
	if (logger) logger(fmt::format("No camera of the wanted type {} found.", wanted_type));
	return {};
}

/// Open an optional camera, or return nothing.
Result<std::optional<NxLibItem>> openCamera(std::optional<NxLibItem> camera) {
	if (!camera) return Error{"could not find camera to open"};

	NxLibCommand command(cmdOpen);
	Result<void> set_result = setNx(command.parameters()[itmCameras], getNx<std::string>((*camera)[itmSerialNumber]));
	if (!set_result) return set_result.error();

	Result<void> execute_result = executeNx(command);
	if (!execute_result) return execute_result.error();

	return camera;
}

Result<std::optional<NxLibItem>> openCameraBySerial(std::string const & serial) {
	return openCamera(findCameraBySerial(serial));
}

Result<std::optional<NxLibItem>> openCameraByEepromId(int eeprom_id) {
	return openCamera(findCameraByEepromId(eeprom_id));
}

Result<std::optional<NxLibItem>> openCameraByLink(std::string const & serial, LogFunction logger) {
	return openCamera(findCameraByLink(serial, logger));
}

Result<std::optional<NxLibItem>> openCameraByType(std::string const & type, LogFunction logger) {
	return openCamera(findCameraByType(type, logger));
}

Result<void> executeNx(NxLibCommand const & command, std::string const & what) {
	int error = 0;
	command.execute(&error);
	if (error) return Error{what};
	return estd::in_place_valid;
}

Result<std::int64_t> getNxBinaryTimestamp(NxLibItem const & item, std::string const & what) {
	int error = 0;
	double timestamp = 0;
	item.getBinaryDataInfo(&error, nullptr, nullptr, nullptr, nullptr, nullptr, &timestamp);
	if (error) return Error(what);
	return (timestamp - 11644473600.0) * 1e6; // Correct for epoch and turn into microseconds.
}

Result<void> setNxJson(NxLibItem const & item, std::string const & json, std::string const & what) {
	int error = 0;
	item.setJson(&error, json, true);
	if (error) return Error(what);
	return estd::in_place_valid;
}

Result<bool> setNxJsonFromFile(NxLibItem const & item, std::string const & filename, std::string const & what) {
	std::ifstream file;
	file.open(filename);

	if (!file.good()) {
		return false;
	}

	file.exceptions(std::ios::failbit | std::ios::badbit);

	std::stringstream buffer;
	buffer << file.rdbuf();
	Result<void> set_nx_result = setNxJson(item, buffer.str(), what);
	if (!set_nx_result) return set_nx_result.error();

	return true;
}

Result<std::string> getNxJson(NxLibItem const & item, std::string const & what) {
	int error = 0;
	std::string result = item.asJson(&error, true);
	if (error) return Error(what);
	return result;
}

Result<void> writeNxJsonToFile(NxLibItem const & item, std::string const & filename, std::string const & what) {
	std::ofstream file;
	file.exceptions(std::ios::failbit | std::ios::badbit);
	file.open(filename);
	Result<std::string> get_nx_json_result = getNxJson(item, what);
	if (!get_nx_json_result) {
		file.close();
		return get_nx_json_result.error();
	}

	file << *get_nx_json_result;
	return estd::in_place_valid;
}

}
