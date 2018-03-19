cmake_minimum_required(VERSION 2.8)
Project(LibFloorPlan)
get_filename_component(PROJECT_ROOT . ABSOLUTE)
# You can prepare verbose Makefile to see what's going.
set(CMAKE_VERBOSE_MAKEFILE false)
# Set build type
# set(CMAKE_BUILD_TYPE Debug)


# set the output directory.
SET(OUTPUT ${PROJECT_ROOT}/output CACHE PATH "Path prefix for installation")
SET(CMAKE_BINARY_DIR ${OUTPUT})

add_subdirectory(./src/lib)
add_subdirectory(./src/test)
add_subdirectory(./src/genmatlab)
# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
            ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM
            )
endif(DOXYGEN_FOUND)
