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

find_package(jsoncpp REQUIRED)
get_target_property(JSON_INC_PATH jsoncpp_lib INTERFACE_INCLUDE_DIRECTORIES)
include_directories(${JSON_INC_PATH})

# Add libraries
set(MAP_LIB_SRC
    src/square_grid.cpp  
)

add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map ltl vis utility cbba jsoncpp_lib gplib auto_vehicle)

target_include_directories(map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


add_executable(test_grid_map test/test_grid_map.cpp)
target_link_libraries(test_grid_map vis map utility ${OpenCV_LIBS})

add_executable(test_uncertain_grid_map test/test_uncertain_grid_map.cpp)
target_link_libraries(test_uncertain_grid_map vis map)


