# Install script for directory: /home/jodie/Workspace/ipas_demo/src/jsoncpp/include

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/jsoncpp/json/features.h;/usr/local/include/jsoncpp/json/reader.h;/usr/local/include/jsoncpp/json/config.h;/usr/local/include/jsoncpp/json/writer.h;/usr/local/include/jsoncpp/json/assertions.h;/usr/local/include/jsoncpp/json/autolink.h;/usr/local/include/jsoncpp/json/value.h;/usr/local/include/jsoncpp/json/json.h;/usr/local/include/jsoncpp/json/forwards.h;/usr/local/include/jsoncpp/json/version.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/jsoncpp/json" TYPE FILE FILES
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/features.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/reader.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/config.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/writer.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/assertions.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/autolink.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/value.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/json.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/forwards.h"
    "/home/jodie/Workspace/ipas_demo/src/jsoncpp/include/json/version.h"
    )
endif()

