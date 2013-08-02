################################################################################
#
# Project settings
#
################################################################################

#
# Minimum required version of cmake 
#
CMAKE_MINIMUM_REQUIRED( VERSION 2.8 )

#
# Project name and programming languages used
#
PROJECT( CRANE_NMPC_EMBOCON CXX C )

#
# Folder path for generated executables
#
SET( EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/.. )

#
# Folder path for generated libraries
#
SET( LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib )

#
# CMake module(s) path
#
SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )

################################################################################
#
# Prerequisites
#
################################################################################

FIND_PACKAGE( ACADO REQUIRED )

################################################################################
#
# User configuration
#
################################################################################

IF( NOT CMAKE_VERBOSE_MAKEFILE )
	SET( CMAKE_VERBOSE_MAKEFILE OFF )
ENDIF( NOT CMAKE_VERBOSE_MAKEFILE )

#
# Build type
#
IF( NOT CMAKE_BUILD_TYPE )
	SET(CMAKE_BUILD_TYPE Debug CACHE STRING
		"Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
		FORCE
	)
ENDIF( NOT CMAKE_BUILD_TYPE )

################################################################################
#
# Compiler settings
#
################################################################################

# TODO

################################################################################
#
# Include directories
#
################################################################################

INCLUDE_DIRECTORIES(
	.
	${ACADO_INCLUDE_DIRS}
)

################################################################################
#
# Build the NMPC generator
#
################################################################################

ADD_EXECUTABLE(
	crane_mpc_generator

	crane_mpc_generator.cpp
)
TARGET_LINK_LIBRARIES(
	crane_mpc_generator

	${ACADO_SHARED_LIBRARIES}
)
SET_TARGET_PROPERTIES(
	crane_mpc_generator

	PROPERTIES
		RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)