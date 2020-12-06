cmake_minimum_required(VERSION 3.5.0)
project(jsbsim)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake-modules")
INCLUDE("${CMAKE_CURRENT_LIST_DIR}/../cmake-modules/CommonSetup.cmake")
CommonSetup()

# don't know what this is for?
IF(UNIX)
    set(JSBSIM_DEFINES " -D MSGPACK_PP_VARIADICS_MSVC=0")
    set(CMAKE_CXX_FLAGS "${JSBSIM_DEFINES} ${CMAKE_CXX_FLAGS}")
ENDIF()

add_subdirectory("${AIRSIM_ROOT}/external/jsbsim/${JSBSIM_VERSION_FOLDER}" "${CMAKE_CURRENT_BINARY_DIR}/jsbsim")
