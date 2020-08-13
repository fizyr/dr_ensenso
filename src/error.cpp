#include "error.hpp"
#include "util.hpp"

namespace dr {

namespace {
	std::string makeErrorMsg(NxLibException const & error, std::string const & what) {
		return what + (what.empty() ? "" : ": ") + "NxLibException at " + error.getItemPath() + ": " + std::to_string(error.getErrorCode()) + ": " + error.getErrorText();
	}

	std::string makeCommandErrorMsg(std::string const & command, std::string const & symbol, std::string const & text, std::string const & what) {
		return what + (what.empty() ? "" : ": ") + "failed to execute NxLibCommand " + command + ": error " + symbol + ": " + text;
	}
}

NxError::NxError(std::string const & path, int error, std::string const & what) : NxError({path,      error}, what) {}
NxError::NxError(NxLibItem const & item,   int error, std::string const & what) : NxError({item.path, error}, what) {}
NxError::NxError(NxLibException const & error, std::string const & what) : NxLibException(error), std::runtime_error(makeErrorMsg(error, what)) {}

NxCommandError::NxCommandError(std::string const & command, std::string const & error_symbol, std::string const & error_text, std::string const & what) :
	runtime_error(makeCommandErrorMsg(command, error_symbol, error_text, what)),
	command_(command),
	error_symbol_(error_symbol),
	error_text_(error_text),
	what_(what) {}

Result<NxCommandError> NxCommandError::convertCommandResult(NxLibItem const & result, std::string const & what) {
	Result<std::string> result_command = getNx<std::string>(result[itmExecute][itmCommand]);
	if (!result_command) return result_command.error().push_description("cant find command item: ");

	Result<std::string> result_error_symbol = getNx<std::string>(result[itmErrorSymbol]);
	if (!result_error_symbol) return result_error_symbol.error().push_description("cant find error symbol: ");

	Result<std::string> result_error_text = getNx<std::string>(result[itmErrorText]);
	if (!result_error_text) return result_error_text.error().push_description("cant find errot text: ");

	return NxCommandError(
		*result_command,
		*result_error_symbol,
		*result_error_text,
		what
	);
}

}
