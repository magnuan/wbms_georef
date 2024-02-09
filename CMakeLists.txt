cmake_minimum_required(VERSION 3.1.0)

project(wbms_georef)

# configure version
set(wbms_georef_VERSION_MAJOR "1")
set(wbms_georef_VERSION_MINOR "2")
set(wbms_georef_VERSION_PATCH "1")
set(wbms_georef_VERSION_SUFFIX "Development")

# options
# See Issue 297.
#option(FORCE_SSE "Compile with SSE instruction only" ON)
#option(FORCE_SSSE3 "Compile with SSSE3 instruction only" OFF)
#option(FORCE_SSE41 "Compile with SSE4.1 instruction only" OFF)
#option(OPTIONAL_TEST "Compile optional test (might be broken)" OFF)
# UNITTEST option must be after CMAKE_BUILT_TYPE

# where cmake find custom modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

# fix c standard used on the project
set(CMAKE_C_STANDARD 99)

# Set additional project information
set(COMPANY "lb2oj")
add_definitions("-DCOMPANY=\"${COMPANY}\"")
set(APPLICATION_NAME "WBMS georef")
add_definitions("-DAPPLICATION_NAME=\"${APPLICATION_NAME}\"")
set(APPLICATION_MAINTAINER="Magnus Andersen, LB2OJ")
set(COPYRIGHT "Copyright (c) 2022 Magnus Andersen, LB2OJ. All rights reserved.")
add_definitions("-DCOPYRIGHT=\"${COPYRIGHT}\"")
set(IDENTIFIER "com.${COMPANY}.${APPLICATION_NAME}")
add_definitions("-DIDENTIFIER=\"${IDENTIFIER}\"")
# raspberry as only lxterminal not xterm
if(NOT (WIN32 OR CYGWIN))
  find_program(BINARY_TERMINAL_BIN lxterminal)
  if(BINARY_TERMINAL_BIN)
    set(APPLICATION_DESKTOP_EXEC "${BINARY_TERMINAL_BIN} -e ${CMAKE_PROJECT_NAME}")
  else()
    set(APPLICATION_DESKTOP_EXEC "xterm -e ${CMAKE_PROJECT_NAME}")
  endif()
endif()

find_package(Git)
if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git/")
  # we can also use `git describe --tags`
  execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE res
    OUTPUT_VARIABLE out
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT res)
    string(REGEX REPLACE "^v([0-9]+)\.([0-9]+)\.([0-9]+)-" "" git_commit ${out})
    set(wbms_georef_VERSION_SUFFIX "-${git_commit}")
    set(wbms_georef_VERSION_COMMIT "${git_commit}")
  endif()
endif()

# set variables
set(wbms_georef_VERSION "${wbms_georef_VERSION_MAJOR}.${wbms_georef_VERSION_MINOR}.${wbms_georef_VERSION_PATCH}-${wbms_georef_VERSION_SUFFIX}")
message(STATUS "${APPLICATION_NAME} Version: ${wbms_georef_VERSION}")
add_definitions("-DWBMS_GEOREF_VERSION=\"${wbms_georef_VERSION}\"")
add_definitions("-DMAJOR_VERSION=${wbms_georef_VERSION_MAJOR}")
add_definitions("-DMINOR_VERSION=${wbms_georef_VERSION_MINOR}")
if(wbms_georef_VERSION_COMMIT)
  add_definitions("-DEXTRA_VERSION=${wbms_georef_VERSION_COMMIT}")
endif()

set(CUSTOM_SRC_DIR "${CMAKE_SOURCE_DIR}/src")
set(CUSTOM_EXTERNAL_DIR "${CMAKE_SOURCE_DIR}/external")
set(CUSTOM_DOC_DIR "${CMAKE_SOURCE_DIR}/doc")
set(CUSTOM_MAN_DIR "${CMAKE_SOURCE_DIR}/man")
set(CUSTOM_GEOTRANZ_DIR "${CUSTOM_EXTERNAL_DIR}/geotranz")
set(CUSTOM_XGETOPT_DIR "${CUSTOM_EXTERNAL_DIR}/xgetopt")
set(CUSTOM_SHELL_SHABANG "#!/bin/sh -e")

# cpack variables
set(CPACK_GENERATOR "ZIP")
set(CPACK_STRIP_FILES true)
set(CPACK_PACKAGE_NAME "${CMAKE_PROJECT_NAME}")
# This has architecture of the build machine, not the target platform.
# e.g. Comes out as x86_64 when building for i686 target platform.
#set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${wbms_georef_VERSION}_${CMAKE_SYSTEM_PROCESSOR}")
# We don't know the target yet so this is set after FindCPUflags.
set(CPACK_PACKAGE_CONTACT "https://github.com/lb2oj/wbms_georef")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "WBMS georef is a georeferencer tool for WBMS sonar data")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_SOURCE_IGNORE_FILES "${PROJECT_BINARY_DIR};/.git/;.gitignore;menu.yml;.travis.yml;.appveyor.yml;default.nix;.envrc;TODOs.org;/.scripts/")
SET(CPACK_PACKAGE_VERSION "${wbms_georef_VERSION}")
SET(CPACK_PACKAGE_VERSION_MAJOR "${wbms_georef_VERSION_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${wbms_georef_VERSION_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${wbms_georef_VERSION_PATCH}")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libproj")

# if we don't set build_type
if(NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type set to: ${CMAKE_BUILD_TYPE}")
message("CMake system: ${CMAKE_SYSTEM_NAME}")

# Unittest should be on for dev builds and off for releases.
if(CMAKE_BUILD_TYPE MATCHES "Release")
  option(UNITTEST "Build unittest binaries." OFF)
else()
  option(UNITTEST "Build unittest binaries." ON)
endif()

# set compiler
include(FindCompiler)

# find cpu flags (and set compiler)
include(FindCPUflags)

if(${ARCHITECTURE} MATCHES "x86")
  set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${wbms_georef_VERSION}_i686")
else()
  set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${wbms_georef_VERSION}_${ARCHITECTURE}")
endif()

# auto include current directory
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# set OS dependent variables
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(LINUX TRUE)

  configure_file("${CMAKE_SOURCE_DIR}/cmake/cpack/${CMAKE_PROJECT_NAME}.desktop.in"
    "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.desktop" @ONLY)

elseif(${CMAKE_SYSTEM_NAME} STREQUAL "FreeBSD")
  set(FREEBSD TRUE)
  configure_file("${CMAKE_SOURCE_DIR}/cmake/cpack/${CMAKE_PROJECT_NAME}.desktop.in"
    "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.desktop" @ONLY)

elseif(${CMAKE_SYSTEM_NAME} STREQUAL "OpenBSD")
  set(OPENBSD TRUE)
  set(HAVE_SNDIO TRUE)

elseif(APPLE)
  if("${CMAKE_OSX_DEPLOYMENT_TARGET}" STREQUAL "")
    message(STATUS "Build for macOS target: local version")
  else()
    message(STATUS "Build for macOS target: ${CMAKE_OSX_DEPLOYMENT_TARGET}")
  endif()

  # prepend path to find_*()
  set(CMAKE_FIND_ROOT_PATH "/opt/local")

  set(CMAKE_MACOSX_RPATH ON)
  message(STATUS "RPATH support: ${CMAKE_MACOSX_RPATH}")

elseif (WIN32)
  if(NOT MSVC)
    message(FATAL_ERROR "You must use Microsoft Visual Studio 2015 or 2017 as compiler")
  endif()
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -D_CRT_SECURE_NO_WARNINGS")
  # compile with full multicore
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /MP")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")

  set(CUSTOM_SHELL_BIN "")
endif()

find_package(Proj)

if (C_CLANG OR C_GCC)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wunused-variable -D_GNU_SOURCE ${EXTRA_FLAGS}")
  #
  # -lm is needed for functions in math.h
  if(PROJ_FOUND)
    message("Linking with libproj")
    link_libraries("-lproj -lm")
  else()
    link_libraries("-lm")
  endif()
elseif (C_MSVC)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -W3 -MP ${EXTRA_FLAGS}")
endif()

if (C_CLANG)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ferror-limit=1")
elseif (C_GCC)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmax-errors=1")
endif()

# set installation directories
if (WIN32 OR CYGWIN)
  set(INSTALL_BIN_DIR ".")
  set(INSTALL_DOC_DIR "doc")
  set(INSTALL_CONF_DIR ".")
  set(INSTALL_SCRIPTS_DIR "scripts")
  set(INSTALL_MAN_DIR "man")
  set(INSTALL_DATA_DIR "data")
else()
  set(INSTALL_BIN_DIR "bin")
  set(INSTALL_DOC_DIR "share/doc/${CMAKE_PROJECT_NAME}")
  set(INSTALL_CONF_DIR "${INSTALL_DOC_DIR}/conf")
  set(INSTALL_SCRIPTS_DIR "${INSTALL_DOC_DIR}/scripts")
  if(FREEBSD)
    set(INSTALL_MAN_DIR "man/man1")
  else()
    set(INSTALL_MAN_DIR "share/man/man1")
  endif()
  set(INSTALL_DATA_DIR "share/${PROJECT_NAME}")
endif(WIN32 OR CYGWIN)

# requirements

if(PROJ_FOUND)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_PROJ")
else()
  set(PROJ_INCLUDE_DIRS "")
  set(PROJ_LIBRARY "")
endif()

# ENABLE PROFILING
#  With this flag enabled, running the binary wil generate a file gmon.out,
#  that can be investigated with gprof
#  gprof ~/git/wbms_georef/build/src/wbms_georef gmon.out >analysis.txt
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pg")

include(CheckSymbolExists)

# external libraries
add_subdirectory(${CUSTOM_GEOTRANZ_DIR})
if (WIN32 OR CYGWIN)
add_subdirectory(${CUSTOM_XGETOPT_DIR})
endif(WIN32 OR CYGWIN)

# wbms_georef source code and utilities
add_subdirectory(src)

# install basic docs
install(FILES ${CMAKE_SOURCE_DIR}/CHANGES.md DESTINATION ${INSTALL_DOC_DIR})
install(FILES ${CMAKE_SOURCE_DIR}/LICENSE DESTINATION ${INSTALL_DOC_DIR})
install(FILES ${CMAKE_SOURCE_DIR}/external/LICENSE DESTINATION ${INSTALL_DOC_DIR}/external)
add_subdirectory(doc)
add_subdirectory(man)

############ uninstall target ################
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/include/uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P
  ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake)

############ packaging ################
add_subdirectory(cmake/cpack)
