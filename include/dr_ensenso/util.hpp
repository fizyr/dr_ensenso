#pragma once
#include "error.hpp"

#include <ensenso/nxLib.h>

#include <boost/optional.hpp>

#include <string>
#include <stdexcept>
#include <cstdint>


namespace dr {

/// Find a camera by serial number.
boost::optional<NxLibItem> findCameraBySerial(std::string const & serial);

/// Find a camera by eeprom ID.
boost::optional<NxLibItem> findCameraByEepromId(int eeprom_id);

/// Find a camera that is linked to another camera given by serial.
boost::optional<NxLibItem> findCameraByLink(std::string const & serial);

/// Find a camera by type.
/**
 * If multiple cameras of the requested type are available, it is unspecified which one will be selected.
 *
 * \return An NxLibItem representing a camera of the requested type if one can be found.
 */
boost::optional<NxLibItem> findCameraByType(std::string const & type);

/// Find and open a camera by serial number.
/**
 * \return The NxLibItem representing the found camera or an empty optional.
 * \throws if opening the camera fails.
 */
boost::optional<NxLibItem> openCameraBySerial(std::string const & serial);

/// Find and open a camera by eeprom ID.
/**
 * \return The NxLibItem representing the found camera or an empty optional.
 * \throws if opening the camera fails.
 */
boost::optional<NxLibItem> openCameraByEepromId(int eeprom_id);

/// Find and open a camera that is linked to another camera given by serial.
/**
 * \return The NxLibItem representing the found camera or an empty optional.
 * \throws if opening the camera fails.
 */
boost::optional<NxLibItem> openCameraByLink(std::string const & serial);

/// Find and open a camera by type.
/**
 * If multiple cameras of the requested type are available, it is unspecified which one will be selected.
 * \return An NxLibItem representing a camera of the requested type if one can be found.
 * \throws if opening the camera fails.
 */
boost::optional<NxLibItem> openCameraByType(std::string const & type);

/// Execute an NxLibCommand.
/**
 * \throw NxError on failure.
 */
void executeNx(NxLibCommand const & command, std::string const & what = "");

/// Get the value of an NxLibItem as the specified type.
/**
 * \throw NxError on failure.
 */
template<typename T>
T getNx(NxLibItem const & item, std::string const & what = "") {
	int error = 0;
	T result = item.as<T>(&error);
	if (error) throw NxError(item, error, what);
	return result;
}

/// Get the timestamp of a binary node as microseconds since January 1 1970 UTC.
std::int64_t getNxBinaryTimestamp(NxLibItem const & item, std::string const & what = "");

/// Set the value of an NxLibItem.
/**
 * \throw NxError on failure.
 */
template<typename T>
void setNx(NxLibItem const & item, T const & value, std::string const & what = "") {
	int error = 0;
	item.set(&error, value);
	if (error) throw NxError(item, error, what);
}

/// Set the value of an NxLibItem to a JSON tree.
/**
 * \throw NxError on failure.
 */
void setNxJson(NxLibItem const & item, std::string const & json, std::string const & what = "");

/// Set the value of an NxLibItem to a JSON tree from a file.
/**
 * \return False if the file was not good (ie. could not be opened or does not exist).
 * \throw NxError on failure.
 */
bool setNxJsonFromFile(NxLibItem const & item, std::string const & filename, std::string const & what = "");

/// Get the value of an NxLibItem as a JSON tree.
/**
 * \throw NxError on failure.
 */
std::string getNxJson(NxLibItem const & item, std::string const & what = "");

/// Get the value of an NxLibItem as JSON tree and write it to a file.
/**
 * \throw NxError on failure.
 */
void writeNxJsonToFile(NxLibItem const & item, std::string const & filename, std::string const & what = "");

}
