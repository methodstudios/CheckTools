cmake_minimum_required(VERSION 3.12)
project(CheckTools CXX)


# Because our Maya installation system is inconsistent, we have to scan all known locations for headers/libraries
list(APPEND MAYA_KNOWN_DIRS $ENV{REZ_MAYA_ROOT}/usr/autodesk/maya/include/maya)
list(APPEND MAYA_KNOWN_DIRS $ENV{REZ_MAYA_ROOT}/ext/usr/autodesk/maya/include/maya)

find_file(MPX_FOUND
    name MPxCommand.h
    PATHS ${MAYA_KNOWN_DIRS}
    NO_DEFAULT_PATH
)

# ../..
get_filename_component(MAYA_ROOT_DIR ${MPX_FOUND} DIRECTORY)
get_filename_component(MAYA_ROOT_DIR ${MAYA_ROOT_DIR} DIRECTORY)
get_filename_component(MAYA_ROOT_DIR ${MAYA_ROOT_DIR} DIRECTORY)


add_subdirectory(src)
