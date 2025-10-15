cmake_minimum_required(VERSION 3.13.0)

project(wbms_georef)

# ---------------------------
# Version & app metadata
# ---------------------------
# configure version
set(wbms_georef_VERSION_MAJOR "1")
set(wbms_georef_VERSION_MINOR "2")
set(wbms_georef_VERSION_PATCH "3")
set(wbms_georef_VERSION_SUFFIX "Development")

# options (from original; kept as comments)
# See Issue 297.
#option(FORCE_SSE "Compile with SSE instruction only" ON)
#option(FORCE_SSSE3 "Compile with SSSE3 instruction only" OFF)
#option(FORCE_SSE41 "Compile with SSE4.1 instruction only" OFF)
#option(OPTIONAL_TEST "Compile optional test (might be broken)" OFF)
# UNITTEST option must be after CMAKE_BUILD_TYPE

# where cmake find custom modules
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/modules)

# fix c standard used on the project
set(CMAKE_C_STANDARD 99)

# Set additional project information
set(COMPANY "lb2oj")
add_definitions("-DCOMPANY=\"${COMPANY}\"")
set(APPLICATION_NAME "WBMS georef")
add_definitions("-DAPPLICATION_NAME=\"${APPLICATION_NAME}\"")
set(APPLICATION_MAINTAINER "Magnus Andersen, LB2OJ")
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

# ---------------------------
# Git commit suffix (unchanged)
# ---------------------------
find_package(Git)
if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git/")
  execute_process(COMMAND "${GIT_EXECUTABLE}" rev-parse --short HEAD
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE res
    OUTPUT_VARIABLE out
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE)
  if(NOT res)
    string(REGEX REPLACE "^v([0-9]+)\\.([0-9]+)\\.([0-9]+)-" "" git_commit ${out})
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

# ---------------------------
# Packaging (unchanged)
# ---------------------------
set(CPACK_GENERATOR "ZIP")
set(CPACK_STRIP_FILES true)
set(CPACK_PACKAGE_NAME "${CMAKE_PROJECT_NAME}")
set(CPACK_PACKAGE_CONTACT "https://github.com/lb2oj/wbms_georef")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "WBMS georef is a georeferencer tool for WBMS sonar data")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_README "${CMAKE_SOURCE_DIR}/README.md")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
set(CPACK_SOURCE_IGNORE_FILES "${PROJECT_BINARY_DIR};/.git/;.gitignore;menu.yml;.travis.yml;.appveyor.yml;default.nix;.envrc;TODOs.org;/.scripts/")
set(CPACK_PACKAGE_VERSION "${wbms_georef_VERSION}")
set(CPACK_PACKAGE_VERSION_MAJOR "${wbms_georef_VERSION_MAJOR}")
set(CPACK_PACKAGE_VERSION_MINOR "${wbms_georef_VERSION_MINOR}")
set(CPACK_PACKAGE_VERSION_PATCH "${wbms_georef_VERSION_PATCH}")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libproj")

# if we don't set build_type (keep original default)
if(NOT DEFINED CMAKE_BUILD_TYPE OR "${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
message(STATUS "Build type set to: ${CMAKE_BUILD_TYPE}")
message("CMake system: ${CMAKE_SYSTEM_NAME}")

# ---------------------------
# Unit tests default by build type (unchanged)
# ---------------------------
if(CMAKE_BUILD_TYPE MATCHES "Release")
  option(UNITTEST "Build unittest binaries." OFF)
else()
  option(UNITTEST "Build unittest binaries." ON)
endif()

# ---------------------------
# Compiler & CPU flags modules (original)
# ---------------------------
include(FindCompiler)
include(FindCPUflags)

# ---------------------------
# Architecture for package name (original)
# ---------------------------
if(${ARCHITECTURE} MATCHES "x86")
  set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${wbms_georef_VERSION}_i686")
else()
  set(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}-${wbms_georef_VERSION}_${ARCHITECTURE}")
endif()

# auto include current directory
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Export compile_commands.json (handy for tooling)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---------------------------
# OS dependent variables (original)
# ---------------------------
if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
  set(LINUX TRUE)
  configure_file("${CMAKE_SOURCE_DIR}/cmake/cpack/${CMAKE_PROJECT_NAME}.desktop.in"
    "${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.desktop" @ONLY)
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

# ---------------------------
# Dependencies
# ---------------------------
find_package(Proj)

# ---------------------------
# Build / Profiling / Optimization options
# ---------------------------
# Shim for add_link_options on older CMake
if(CMAKE_VERSION VERSION_LESS "3.13")
  function(add_link_options)
    foreach(opt IN LISTS ARGN)
      set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${opt}" PARENT_SCOPE)
      set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${opt}" PARENT_SCOPE)
    endforeach()
  endfunction()
endif()

# User-togglable options (safe defaults)
option(ENABLE_PERF_PROFILE "Add -g and frame pointers for perf sampling" OFF)
option(ENABLE_CALLGRIND_PROFILE "Add -g for Callgrind symbolization" ON)
option(ENABLE_GPROF "Enable -pg instrumentation for gprof (slow)" OFF)
option(ENABLE_LTO "Enable Link-Time Optimization/IPO" ON)
option(ENABLE_NATIVE "Enable -march/-mtune=native (host-specific)" OFF)
option(ENABLE_FAST_MATH "Enable -ffast-math (unsafe transformations)" OFF)
option(ENABLE_PGO "Enable Profile-Guided Optimization" OFF)
set(PGO_STAGE "generate" CACHE STRING "PGO stage: generate|use")
set(PGO_PROFDATA "" CACHE FILEPATH "Path to .profdata (clang)")

# Baseline compiler/linker flags
if (C_CLANG OR C_GCC)
  # Keep your existing warnings and extras
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wunused-variable -D_GNU_SOURCE ${EXTRA_FLAGS}")

  # Friendly error limits (original)
  if (C_CLANG)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -ferror-limit=1")
  elseif (C_GCC)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmax-errors=1")
  endif()

  # Optimization level and NDEBUG
  add_compile_options(-O3 -DNDEBUG)

  # Debug info for profilers
  if(ENABLE_PERF_PROFILE OR ENABLE_CALLGRIND_PROFILE)
    add_compile_options(-g)
    add_link_options(-g)
    # Better symbolization in perf/valgrind
    if(LINUX)
      add_link_options(-rdynamic)
    endif()
  endif()

  # Frame pointers help sampling profilers (tiny runtime cost)
  if(ENABLE_PERF_PROFILE AND LINUX)
    add_compile_options(-fno-omit-frame-pointer)
    add_link_options(-fno-omit-frame-pointer)
  endif()

  # gprof (instrumentation)
  if(ENABLE_GPROF AND LINUX)
    add_compile_options(-pg)
    add_link_options(-pg -no-pie)
  endif()

  # ISA tuning (avoid for cross-compiles/portable packages)
  if(ENABLE_NATIVE)
    add_compile_options(-march=native -mtune=native)
  endif()

  # Numerics: safer fast-math subset; full fast-math optional
  add_compile_options(-fno-math-errno -fno-trapping-math -ffp-contract=fast)
  if(ENABLE_FAST_MATH)
    add_compile_options(-ffast-math)
  endif()

  # Section GC to trim binary size
  add_compile_options(-ffunction-sections -fdata-sections)
  add_link_options(-Wl,--gc-sections -Wl,-O2 -Wl,--as-needed)

  # LTO / IPO
  if(ENABLE_LTO)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.9")
      set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
    else()
      add_compile_options(-flto)
      add_link_options(-flto)
    endif()
  endif()

  # PGO (Clang/GCC)
  if(ENABLE_PGO)
    if(C_CLANG)
      if(PGO_STAGE STREQUAL "generate")
        add_compile_options(-fprofile-instr-generate)
        add_link_options(-fprofile-instr-generate)
      elseif(PGO_STAGE STREQUAL "use" AND PGO_PROFDATA)
        add_compile_options(-fprofile-instr-use=${PGO_PROFDATA})
        add_link_options(-fprofile-instr-use=${PGO_PROFDATA})
      endif()
    else() # GCC
      if(PGO_STAGE STREQUAL "generate")
        add_compile_options(-fprofile-generate)
        add_link_options(-fprofile-generate)
      else()
        add_compile_options(-fprofile-use -fprofile-correction)
        add_link_options(-fprofile-use)
      endif()
    endif()
  endif()
elseif (C_MSVC)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -W3 -MP ${EXTRA_FLAGS}")
endif()

# ---------------------------
# Install directories (original)
# ---------------------------
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
endif()

# ---------------------------
# Requirements / feature defines (original)
# ---------------------------
if(PROJ_FOUND)
  message("Linking with libproj")
  # Global link kept for compatibility with src/ (can be moved to targets)
  link_libraries("-lproj -lm")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DENABLE_PROJ")
else()
  link_libraries("-lm")
  set(PROJ_INCLUDE_DIRS "")
  set(PROJ_LIBRARY "")
endif()

# ---------------------------
# Profiling (original note kept; gprof now toggleable)
# ---------------------------
# With ENABLE_GPROF=ON, running the binary will generate gmon.out for gprof:
# gprof <path-to-binary> gmon.out > analysis.txt

include(CheckSymbolExists)

# ---------------------------
# Subdirectories (original)
# ---------------------------
# external libraries
add_subdirectory(${CUSTOM_GEOTRANZ_DIR})
if (WIN32 OR CYGWIN)
  add_subdirectory(${CUSTOM_XGETOPT_DIR})
endif()

# wbms_georef source code and utilities
add_subdirectory(src)

# install basic docs
install(FILES ${CMAKE_SOURCE_DIR}/CHANGES.md DESTINATION ${INSTALL_DOC_DIR})
install(FILES ${CMAKE_SOURCE_DIR}/LICENSE DESTINATION ${INSTALL_DOC_DIR})
install(FILES ${CMAKE_SOURCE_DIR}/external/LICENSE DESTINATION ${INSTALL_DOC_DIR}/external)
add_subdirectory(doc)
add_subdirectory(man)

# ---------------------------
# Uninstall target (original)
# ---------------------------
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/include/uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  COMMAND ${CMAKE_COMMAND} -P
  ${CMAKE_CURRENT_BINARY_DIR}/uninstall.cmake)

# ---------------------------
# Packaging (original)
# ---------------------------
add_subdirectory(cmake/cpack)

