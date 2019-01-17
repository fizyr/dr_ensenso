include(FindPkgConfig)

find_path(jsoncpp_INCLUDE_DIR
	NAMES json/json.h
	DOC "The jsoncpp include directories."
)

find_library(
	jsoncpp_LIBRARY
	NAMES jsoncpp
	DOC "The jsoncpp libraries."
)

set(jsoncpp_INCLUDE_DIRS "${jsoncpp_INCLUDE_DIR}")
set(jsoncpp_LIBRARIES    "${jsoncpp_LIBRARY}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(jsoncpp
	FOUND_VAR jsoncpp_FOUND
	REQUIRED_VARS jsoncpp_LIBRARIES jsoncpp_INCLUDE_DIRS
)

mark_as_advanced(jsoncpp_FOUND)
