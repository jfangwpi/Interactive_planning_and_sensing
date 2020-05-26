# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)
find_package(LCM REQUIRED)
find_package(Eigen3 3.1.2)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})


# Add libraries
set(VEHICLE_LIB_SRC
    agent.hpp
    agent_impl.hpp
    auto_team.hpp
    auto_team_impl.hpp
)

add_library(vehicle INTERFACE)
target_sources(vehicle INTERFACE ${VEHICLE_LIB_SRC})
target_include_directories(vehicle INTERFACE
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<INSTALL_INTERFACE:include>)


