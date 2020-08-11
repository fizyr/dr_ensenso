#pragma once
#include "error.hpp"
#include "types.hpp"

#include <ensenso/nxLib.h>

#include <cstdint>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>


namespace dr {

using LogFunction = std::function<void (std::string)>;

/// Find a camera by serial number.
std::optional<NxLibItem> findCameraBySerial(std::string const & serial);

/// Find a camera by eeprom ID.
std::optional<NxLibItem> findCameraByEepromId(int eeprom_id);

/// Find a camera that is linked to another camera given by serial.
std::optional<NxLibItem> findCameraByLink(std::string const & serial, LogFunction logger = nullptr);

/// Find a camera by type.
/**
 * If multiple cameras of the requested type are available, it is unspecified which one will be selected.
 *
 * \return An NxLibItem representing a camera of the requested type if one can be found.
 */
std::optional<NxLibItem> findCameraByType(std::string const & type, LogFunction logger = nullptr);

/// Find and open a camera by serial number.
/**
 * \return the NxLibItem representing the found camera or an empty optional.
 */
Result<NxLibItem> openCameraBySerial(std::string const & serial);

/// Find and open a camera by eeprom ID.
/**
 * \return the NxLibItem representing the found camera or an empty optional.
 */
Result<NxLibItem> openCameraByEepromId(int eeprom_id);

/// Find and open a camera that is linked to another camera given by serial.
/**
 * \return the NxLibItem representing the found camera or an empty optional.
 */
Result<NxLibItem> openCameraByLink(std::string const & serial, LogFunction logger = nullptr);

/// Find and open a camera by type.
/**
 * If multiple cameras of the requested type are available, it is unspecified which one will be selected.
 * \return the NxLibItem representing the found camera or an empty optional.
 */
Result<NxLibItem> openCameraByType(std::string const & type, LogFunction logger = nullptr);

/// Execute an NxLibCommand.
Result<void> executeNx(NxLibCommand const & command, std::string const & what = "");

/// Get the value of an NxLibItem as the specified type.
template<typename T>
Result<T> getNx(NxLibItem const & item, std::string const & what = "") {
	int error = 0;
	T result = item.as<T>(&error);
	if (error) {
		NxError nx_error{item, error, what};
		return Error(nx_error.what());
	}
	return result;
}

/// Set the value of an NxLibItem.
template<typename T>
Result<void> setNx(NxLibItem const & item, T const & value, std::string const & what = "") {
	int error = 0;
	item.set(&error, value);
	if (error) {
		NxError nx_error{item, error, what};
		return Error(nx_error.what());
	}
	return estd::in_place_valid;
}

/// Get the timestamp of a binary node as microseconds since January 1 1970 UTC.
Result<std::int64_t> getNxBinaryTimestamp(NxLibItem const & item, std::string const & what = "");

/// Set the value of an NxLibItem to a JSON tree.
Result<void> setNxJson(NxLibItem const & item, std::string const & json, std::string const & what = "");

/// Set the value of an NxLibItem to a JSON tree from a file.
Result<void> setNxJsonFromFile(NxLibItem const & item, std::string const & filename, std::string const & what = "");

/// Get the value of an NxLibItem as a JSON tree.
Result<std::string> getNxJson(NxLibItem const & item, std::string const & what = "");

/// Get the value of an NxLibItem as JSON tree and write it to a file.
Result<void> writeNxJsonToFile(NxLibItem const & item, std::string const & filename, std::string const & what = "");

}
