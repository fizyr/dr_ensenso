#pragma once
#include "error.hpp"
#include "types.hpp"

#include <estd/any.hpp>

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
 * \return a result containing The NxLibItem representing the found camera or an empty optional.
 * \result is error if opening the camera fails.
 */
Result<std::optional<NxLibItem>> openCameraBySerial(std::string const & serial);

/// Find and open a camera by eeprom ID.
/**
 * \return a result containing The NxLibItem representing the found camera or an empty optional.
 * \result is error if opening the camera fails.
 */
Result<std::optional<NxLibItem>> openCameraByEepromId(int eeprom_id);

/// Find and open a camera that is linked to another camera given by serial.
/**
 * \return a result containing The NxLibItem representing the found camera or an empty optional.
 * \result is error if opening the camera fails.
 */
Result<std::optional<NxLibItem>> openCameraByLink(std::string const & serial, LogFunction logger = nullptr);

/// Find and open a camera by type.
/**
 * If multiple cameras of the requested type are available, it is unspecified which one will be selected.
 * \return a result containing The NxLibItem representing the found camera or an empty optional.
 * \result is error if opening the camera fails.
 */
Result<std::optional<NxLibItem>> openCameraByType(std::string const & type, LogFunction logger = nullptr);

/// Execute an NxLibCommand.
/**
 * \result is error if executing Command failed.
 */
Result<void> executeNx(NxLibCommand const & command, std::string const & what = "");

/// Get the value of an NxLibItem as the specified type.
/**
 * Returns Result<T, Error>
 */
template<typename T>
Result<T> getNx(NxLibItem const & item, std::string const & what = "") {
	int error = 0;
	T result = item.as<T>(&error);
	if (error) return Error{what};
	return result;
}

/// Set the value of an NxLibItem.
/**
 * Returns Result<void, Error>
 */
template<typename T>
Result<void> setNx(NxLibItem const & item, T const & value, std::string const & what = "") {
	int error = 0;
	item.set(&error, value);
	if (error) return Error{what};
}

/// Get the timestamp of a binary node as microseconds since January 1 1970 UTC.
Result<std::int64_t> getNxBinaryTimestamp(NxLibItem const & item, std::string const & what = "");

/// Set the value of an NxLibItem to a JSON tree.
/**
 * return Error on failure on failure.
 */
Result<void> setNxJson(NxLibItem const & item, std::string const & json, std::string const & what = "");

/// Set the value of an NxLibItem to a JSON tree from a file.
/**
 * \return False if the file was not good (ie. could not be opened or does not exist).
 * \return Error on failure.
 */
Result<bool> setNxJsonFromFile(NxLibItem const & item, std::string const & filename, std::string const & what = "");

/// Get the value of an NxLibItem as a JSON tree.
/**
 * \return Error on failure.
 */
Result<std::string> getNxJson(NxLibItem const & item, std::string const & what = "");

/// Get the value of an NxLibItem as JSON tree and write it to a file.
/**
 * \return Error on failure.
 */
Result<void> writeNxJsonToFile(NxLibItem const & item, std::string const & filename, std::string const & what = "");

}
