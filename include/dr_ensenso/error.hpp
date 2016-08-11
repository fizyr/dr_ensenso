#pragma once
#include <ensenso/nxLib.h>
#include <stdexcept>

namespace dr {

/// Wrapper for NxLibException that also inherits from std::runtime_error.
class NxError : public NxLibException, public std::runtime_error {
public:
	NxError(std::string const & path, int error, std::string const & what = "");
	NxError(NxLibItem const & item,   int error, std::string const & what = "");
	NxError(NxLibException const & error,        std::string const & what = "");
};

/// An error while executing a command.
class NxCommandError : public std::runtime_error {
	std::string command_;
	std::string error_symbol_;
	std::string error_text_;
	std::string what_;

public:
	/// Construct a command error from a command name, error symbol, error text and an optional extra message.
	NxCommandError(std::string const & command, std::string const & error_symbol, std::string const & error_text, std::string const & what = "");

	/// Make a NxCommandError representing the current command error.
	static NxCommandError getCurrent(std::string const & what = "");

	std::string const & command() const { return command_; }
	std::string const & error_symbol() const { return error_symbol_; }
	std::string const & error_text() const { return error_text_; }
	std::string const & extra() const { return what_; }
};

/// Throw a regular NxError or NxCommandError depending on the error details.
/**
 * Must be called right after command execution so it can retrieve the error details.
 */
void throwCommandError(int error, std::string const & what);

}
