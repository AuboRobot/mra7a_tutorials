# - Try to find IKOptimalHandler
# Once done this will define
#
# libIKOptimalHandler_FOUND - system has libIKOptimalHandler
# libIKOptimalHandler_INCLUDE_DIRS - the libIKOptimalHandler include directories
# libIKOptimalHandler - link these to use libIKOptimalHandler

find_path(libIKOptimalHandler_INCLUDE_DIR
	NAMES IKOptimalHandler.h
	PATHS ${PROJECT_SOURCE_DIR}/include/IKOptimalHandler
	      $ENV{INCLUDE}
              $../include/IKOptimalHandler
)


set(libIKOptimalHandler_INCLUDE_DIRS ${libIKOptimalHandler_INCLUDE_DIR})

find_library(libIKOptimalHandler_LIB
	NAMES IKOptimalHandler
	PATHS ${PROJECT_SOURCE_DIR}/lib/
)

set(libIKOptimalHandler_LIBS ${libIKOptimalHandler_LIB})

if(libIKOptimalHandler_INCLUDE_DIRS)
	message(STATUS "Found IKOptimalHandler include dir: ${libIKOptimalHandler_DIRS}")
else(libIKOptimalHandler_INCLUDE_DIRS)
	message(STATUS "Could NOT find IKOptimalHandler headers.")
endif(libIKOptimalHandler_INCLUDE_DIRS)

if(libIKOptimalHandler_LIBS)
	message(STATUS "Found IKOptimalHandler library: ${libIKOptimalHandler_LIBS}")
else(libIKOptimalHandler_LIBS)
	message(STATUS "Could NOT find libIKOptimalHandler library.")
endif(libIKOptimalHandler_LIBS)

if(libIKOptimalHandler_INCLUDE_DIRS AND libIKOptimalHandler_LIBS)
	set(libIKOptimalHandler_FOUND TRUE)
else(libIKOptimalHandler_INCLUDE_DIRS AND libIKOptimalHandler_LIBS)
	set(libIKOptimalHandler_FOUND FALSE)
	if(libIKOptimalHandler_FIND_REQUIRED)
		message(FATAL_ERROR "Could not find IKOptimalHandler.")
	endif(libIKOptimalHandler_FIND_REQUIRED)
endif(libIKOptimalHandler_INCLUDE_DIRS AND libIKOptimalHandler_LIBS)
