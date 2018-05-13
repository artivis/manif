cmake_minimum_required(VERSION 2.8.8)

# ExternalProject CMake module
include(ExternalProject)

# Download spdlog
ExternalProject_Add(spdlog
  PREFIX spdlog
  #--Download step--------------
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  #--Configure step-------------
  CONFIGURE_COMMAND ""
  #--Build step-----------------
  BUILD_COMMAND ""
  #--Install step---------------
  UPDATE_COMMAND ""
  INSTALL_COMMAND ""
  LOG_DOWNLOAD ON
)

set(LSPDLOG_INCLUDE_DIRS
    ${LSPDLOG_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/spdlog/src/spdlog/include PARENT_SCOPE)
