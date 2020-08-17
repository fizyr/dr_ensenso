#pragma once
#include "types.hpp"

#include <ensenso/nxLib.h>

#include <cstdint>
#include <functional>
#include <optional>
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
Result<void> executeNx(NxLibCommand const & command);

/// Check if an item exists in the NxTree
Result<bool> existsNx(NxLibItem const & item);

// Error functions
std::string getNxErrorName(int error);
std::string getNxErrorDescription(int error);
std::string getNxErrorWithDescription(int error);
std::string composeTreeReadErrorMessage(int error, NxLibItem const & item);
std::string composeTreeWriteErrorMessage(int error, NxLibItem const & item);

Result<std::string> composeCommandErrorMessage(NxLibItem const & result);

/// Get the value of an NxLibItem as the specified type.
template<typename T>
Result<T> getNx(NxLibItem const & item) {
	int error = 0;
	T result = item.as<T>(&error);
	if (error) return estd::error(composeTreeReadErrorMessage(error, item));
	return result;
}

/// Set the value of an NxLibItem.
template<typename T>
Result<void> setNx(NxLibItem const & item, T const & value) {
	int error = 0;
	item.set(&error, value);
	if (error) return estd::error(composeTreeWriteErrorMessage(error, item));
	return estd::in_place_valid;
}

/// Get the timestamp of a binary node as microseconds since January 1 1970 UTC.
Result<std::int64_t> getNxBinaryTimestamp(NxLibItem const & item);

/// Set the value of an NxLibItem to a JSON tree.
Result<void> setNxJson(NxLibItem const & item, std::string const & json);

/// Set the value of an NxLibItem to a JSON tree from a file.
Result<void> setNxJsonFromFile(NxLibItem const & item, std::string const & filename);

/// Get the value of an NxLibItem as a JSON tree.
Result<std::string> getNxJson(NxLibItem const & item);

/// Get the value of an NxLibItem as JSON tree and write it to a file.
Result<void> writeNxJsonToFile(NxLibItem const & item, std::string const & filename);

}
