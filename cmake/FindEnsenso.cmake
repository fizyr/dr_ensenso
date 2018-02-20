include(FindPkgConfig)

find_path(Ensenso_INCLUDE_DIR
	NAMES ensenso/nxLib.h
	PATHS
		/opt/ensenso/include
	DOC "The Ensenso include directories."
)

function (dr_ensenso_make_include_subdir)
	find_path(dr_ensenso_alternative_include_dir
		NAMES nxLib.h
		PATHS /opt/ensenso/development/c/include
		DOC "Alternative Ensenso include directories."
	)
	if (dr_ensenso_alternative_include_dir)
		set(new_include_dir "${CMAKE_CURRENT_BINARY_DIR}/include")
		if (CATKIN_DEVEL_PREFIX)
			set(new_include_dir "${CATKIN_DEVEL_PREFIX}/include")
		endif()

		file(GLOB nxlib_headers RELATIVE "${dr_ensenso_alternative_include_dir}" "${dr_ensenso_alternative_include_dir}/nxLib*")
		file(MAKE_DIRECTORY "${new_include_dir}/ensenso")
		foreach(header IN LISTS nxlib_headers)
			execute_process(COMMAND ln -s "${dr_ensenso_alternative_include_dir}/${header}" "${new_include_dir}/ensenso")
		endforeach()
		set(Ensenso_INCLUDE_DIR "${new_include_dir}" PARENT_SCOPE)
	endif()
endfunction()

if (NOT Ensenso_INCLUDE_DIR)
	dr_ensenso_make_include_subdir()
endif()

find_library(
	Ensenso_LIBRARY
	NAMES NxLib64
	PATHS
		/opt/ensenso/lib
	DOC "The Ensenso libraries."
)

set(Ensenso_INCLUDE_DIRS "${Ensenso_INCLUDE_DIR}" "${Ensenso_INCLUDE_DIR}/ensenso")
set(Ensenso_LIBRARIES    "${Ensenso_LIBRARY}")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Ensenso
	FOUND_VAR Ensenso_FOUND
	REQUIRED_VARS Ensenso_LIBRARIES Ensenso_INCLUDE_DIRS
)

mark_as_advanced(Ensenso_FOUND)
