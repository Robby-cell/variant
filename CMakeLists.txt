#=============================================================================
# CMake Configuration for the 'variant' project
#
# A modern, header-only C++ library project template.
#=============================================================================

cmake_minimum_required(VERSION 3.16)

project(variant
    VERSION 1.0.0
    DESCRIPTION "A custom C++ variant implementation"
    LANGUAGES CXX)

#-----------------------------------------------------------------------------
# Options
#-----------------------------------------------------------------------------
option(VARIANT_BUILD_EXAMPLES "Build example programs." OFF)
option(VARIANT_BUILD_TESTS "Build the tests for this project." OFF)

#-----------------------------------------------------------------------------
# Project Configuration
#-----------------------------------------------------------------------------

#-----------------------------------------------------------------------------
# Library Target
#
# We define our library as an INTERFACE library, which is the correct
# type for a header-only library. It has no source files to compile;
# it only "exports" its properties (include directories, compile features)
# to consumers.
#-----------------------------------------------------------------------------
add_library(variant INTERFACE)

# Add the 'include' directory to the public interface. Consumers will
# automatically get this include path when they link against 'variant'.
target_include_directories(variant INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # Path relative to CMAKE_INSTALL_PREFIX
)

# Specify that any target linking against 'variant' must also use C++17.
# This is a "usage requirement".
target_compile_features(variant INTERFACE cxx_std_17)

#-----------------------------------------------------------------------------
# Testing (gtest)
#-----------------------------------------------------------------------------
if(VARIANT_BUILD_TESTS)
    add_subdirectory(tests)
endif()

#-----------------------------------------------------------------------------
# Examples
#-----------------------------------------------------------------------------
if(VARIANT_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# Use standard installation directory variables (e.g., CMAKE_INSTALL_INCLUDEDIR)
include(GNUInstallDirs)

# Install the header files.
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This creates a "variant-targets.cmake" file which contains the definition
# of our 'variant' INTERFACE target.
install(TARGETS variant
    EXPORT VariantTargets
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/variant
)

# Generate and install the package configuration and version files.
# These are the files that `find_package` looks for.
include(CMakePackageConfigHelpers)

# Generate a `variant-config-version.cmake` file.
write_basic_package_version_file(
    "variant-config-version.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)

# Generate a `variant-config.cmake` file.
configure_package_config_file(
    "cmake/variant-config.cmake.in" # Input template file
    "variant-config.cmake"          # Output file
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/variant
)

# Install the generated config and version files.
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/variant-config.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/variant-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/variant
)

# Install the exported target definition file.
install(
    EXPORT VariantTargets
    FILE VariantTargets.cmake
    NAMESPACE variant::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/variant
)

# Display a summary message after configuration
message(STATUS "---------------------------------------------------------")
message(STATUS "Project:            ${PROJECT_NAME}")
message(STATUS "Version:            ${PROJECT_VERSION}")
message(STATUS "Build tests:        ${VARIANT_BUILD_TESTS}")
message(STATUS "Build examples:     ${VARIANT_BUILD_EXAMPLES}")
message(STATUS "Install path:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "---------------------------------------------------------")
