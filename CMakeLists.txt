# Olivier Stasse, Francois Keith, Nicolas Perrin JRL, CNRS/AIST
# Creation: 31/10/2008
# History:
#
# Copyright CNRS/AIST

cmake_minimum_required(VERSION 2.6)

enable_language(CXX)

# Debug mode for cmake.
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Set version
SET(PROJECT_VERSION 0.9.99)

IF(WIN32)
  SET(LIBDIR_KW "/LIBPATH:")
  SET(LIBINCLUSION_KW "")
  SET(LIBINCLUSION_EXT ".lib")
ENDIF(WIN32)

IF(UNIX)
  SET(LIBDIR_KW "-L")
  SET(LIBINCLUSION_KW "-l")
  SET(LIBINCLUSION_EXT "")
ENDIF(UNIX)

# name of the project
SET(PROJECT_NAME analyticalPG)
SET(PROJECT_CONTACT )
SET(PROJECT_DESCRIPTION "")
project(${PROJECT_NAME})

#SET(CMAKE_INSTALL_PREFIX "./build/")

# Impose shared libraries
#------------------------
OPTION(BUILD_SHARED_LIBS "Build it with shared libraries." ON)

SET(${PROJECT_NAME}_CXXFLAGS "-Wall ${${PROJECT_NAME}_CXXFLAGS}")

SET(${PROJECT_NAME}_src_LDFLAGS "${LIBDIR_KW}${CMAKE_PREFIX_INSTALL}/lib")
SET(${PROJECT_NAME}_LDFLAGS "${LIBDIR_KW}${CMAKE_PREFIX_INSTALL}/lib ${LIBINCLUSION_KW}${PROJECT_NAME}-${PROJECT_VERSION}${LIBINCLUSION_EXT} ${${PROJECT_NAME}_LDFLAGS}")

# -------------------------

add_subdirectory(src)
add_subdirectory(unitTesting)

# Prepare the documentation
# -------------------------
INCLUDE(FindDoxygen)
IF(DOXYGEN_FOUND)
  MESSAGE(STATUS "Doxygen found")
  add_subdirectory(doc)
ENDIF(DOXYGEN_FOUND)

# Prepare description file for pkg-config.
#-----------------------------------------

SET(install_pkg_prefix "\${prefix}")
SET(install_pkg_exec_prefix "\${exec_prefix}")
SET(install_pkg_libdir "\${libdir}")
SET(install_pkg_include_dir "\${includedir}")

# Install the file in the appropriate place.
SET(install_libdir_pkgconfig ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

CONFIGURE_FILE(${${PROJECT_NAME}_SOURCE_DIR}/${PROJECT_NAME}.pc.cmake
		 ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc )

INSTALL(FILES ${${PROJECT_NAME}_BINARY_DIR}/${PROJECT_NAME}.pc
	DESTINATION ${install_libdir_pkgconfig}	
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE )






