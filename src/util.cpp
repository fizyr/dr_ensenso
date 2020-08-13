#include "util.hpp"

#include <fstream>
#include <sstream>

#include <fmt/format.h>

namespace dr {

std::optional<NxLibItem> findCameraBySerial(std::string const & serial) {
	NxLibItem camera = NxLibItem{}[itmCameras][serial];
	Result<bool> camera_exists = existsNx(camera);
	if (!camera_exists || !*camera_exists) {
		return {};
	}
	return camera;
}

std::optional<NxLibItem> findCameraByEepromId(int eeprom_id) {
	NxLibItem camera = NxLibItem{}[itmCameras][itmByEepromId][eeprom_id];\
	Result<bool> camera_exists = existsNx(camera);
	if (!camera_exists || !*camera_exists) {
		return {};
	}
	return camera;
}

std::optional<NxLibItem> findCameraByLink(std::string const & linked_to, LogFunction logger) {
	if (logger) logger(fmt::format("Looking for camera linked to target {}.", linked_to));
	NxLibItem cameras = NxLibItem{}[itmCameras];
	for (int i = 0; i < cameras.count(); ++i) {
		Result<std::string> serial = getNx<std::string>(cameras[i][itmSerialNumber]);
		if (!serial) continue;

		NxLibItem camera = cameras[*serial];
		Result<bool> camera_target_exists = existsNx(camera[itmLink][itmTarget]);
		if (!camera_target_exists || !*camera_target_exists) {
			if (logger) logger(fmt::format("Camera with serial {} is not linked to anything."));
			continue;
		}
		Result<std::string> target = getNx<std::string>(camera[itmLink][itmTarget]);
		if (!target) continue;

		if (target == linked_to) {
			if (logger) logger(fmt::format("Camera with serial {} is linked our target: {}", *serial, *target));
			return camera;
		} else {
			if (logger) logger(fmt::format("Camera with serial {} is not linked to our target: {}", *serial, *target));
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
			if (logger) logger(fmt::format("Camera with serial {} is the wanted type: {}", *serial, *type));
			return camera;
		} else {
			if (logger) logger(fmt::format("Camera with serial {} is not the wanted type: {}", *serial, *type));
		}
	}
	if (logger) logger(fmt::format("No camera of the wanted type {} found.", wanted_type));
	return {};
}

/// Open an optional camera, or return nothing.
Result<NxLibItem> openCamera(NxLibItem camera) {
	NxLibCommand command(cmdOpen);

	Result<std::string> serial_number = getNx<std::string>((camera)[itmSerialNumber]);
	if (!serial_number) return serial_number.error().push_description("failed to open camera, serial number not found: ");

	Result<void> set_camera_param = setNx(command.parameters()[itmCameras], *serial_number);
	if (!set_camera_param) return set_camera_param.error().push_description("failed to open camera, could not set camera parameter: ");

	Result<void> execute_open = executeNx(command);
	if (!execute_open) return execute_open.error().push_description("failed execute open command");

	return camera;
}

Result<NxLibItem> openCameraBySerial(std::string const & serial) {
	std::optional<NxLibItem> camera = findCameraBySerial(serial);
	if (!camera) return estd::error(fmt::format("could not find camera with serial {}", serial));

	return openCamera(camera.value());
}

Result<NxLibItem> openCameraByEepromId(int eeprom_id) {
	std::optional<NxLibItem> camera = findCameraByEepromId(eeprom_id);
	if (!camera) return estd::error(fmt::format("could not find camera with eeprom_id {}", eeprom_id));

	return openCamera(camera.value());
}

Result<NxLibItem> openCameraByLink(std::string const & serial, LogFunction logger) {
	std::optional<NxLibItem> camera = findCameraByLink(serial, logger);
	if (!camera) return estd::error(fmt::format("could not find camera by link {}", serial));

	return openCamera(camera.value());
}

Result<NxLibItem> openCameraByType(std::string const & type, LogFunction logger) {
	std::optional<NxLibItem> camera = findCameraByType(type, logger);
	if (!camera) return estd::error(fmt::format("could not find camera by type {}", type));

	return openCamera(camera.value());
}

Result<void> executeNx(NxLibCommand const & command, std::string const & what) {
	int error = 0;
	command.execute(&error);
	if (error) {
		if (error == NxLibExecutionFailed) {
			Result<NxCommandError> nx_command_error = NxCommandError::convertCommandResult(command.result());
			// Could not construct the nxCommandError
			if (!nx_command_error) {
				return nx_command_error.error().push_description("failed to construct command error");
			}

			return Error(nx_command_error->what());
		}
		return Error(NxError{itmExecute, error, what}.what());
	}
	return estd::in_place_valid;
}

Result<bool> existsNx(NxLibItem const & item, std::string const & what) {
	int error = 0;
	bool result = item.exists(&error);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}
	return result;
}

Result<std::int64_t> getNxBinaryTimestamp(NxLibItem const & item, std::string const & what) {
	int error = 0;
	double timestamp = 0;
	item.getBinaryDataInfo(&error, nullptr, nullptr, nullptr, nullptr, nullptr, &timestamp);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}
	return (timestamp - 11644473600.0) * 1e6; // Correct for epoch and turn into microseconds.
}

Result<void> setNxJson(NxLibItem const & item, std::string const & json, std::string const & what) {
	int error = 0;
	item.setJson(&error, json, true);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}
	return estd::in_place_valid;
}

Result<void> setNxJsonFromFile(NxLibItem const & item, std::string const & filename, std::string const & what) {
	std::ifstream file;
	file.open(filename);

	if (!file.good()) return estd::error("failed to set json params from file, the file could not be openend or does not exist");

	file.exceptions(std::ios::failbit | std::ios::badbit);

	//TODO: put in try catch block
	std::stringstream buffer;
	buffer << file.rdbuf();

	Result<void> set_json = setNxJson(item, buffer.str(), what);
	if (!set_json) return set_json.error().push_description("failed to set json params from file: ");

	return estd::in_place_valid;
}

Result<std::string> getNxJson(NxLibItem const & item, std::string const & what) {
	int error = 0;
	std::string result = item.asJson(&error, true);
	if (error) {
		return Error(NxError{item, error, what}.what());
	}
	return result;
}

Result<void> writeNxJsonToFile(NxLibItem const & item, std::string const & filename, std::string const & what) {
	std::ofstream file;
	file.exceptions(std::ios::failbit | std::ios::badbit);
	file.open(filename);
	Result<std::string> json = getNxJson(item, what);
	if (!json) {
		return json.error();
	}

	//TODO: put in try catch block
	file << *json;
	return estd::in_place_valid;
}

}
