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

add_executable(test_ipas_cbba test/test_ipas_cbba.cpp)
target_link_libraries(test_ipas_cbba vis map cbba ltl ${LCM_LIBRARIES})

add_executable(test_ig_standard test/test_ig_standard.cpp)
target_link_libraries(test_ig_standard vis map cbba ltl ${LCM_LIBRARIES})

add_executable(test_data_analysis test/test_data_analysis.cpp)
target_link_libraries(test_data_analysis vis map cbba ltl ${LCM_LIBRARIES})

add_executable(test_ipas_simulation test/test_ipas_simulation.cpp)
target_link_libraries(test_ipas_simulation vis map cbba ltl ${LCM_LIBRARIES} jsoncpp_lib)

add_executable(test_random_map_availability test/test_random_map_availability.cpp)
target_link_libraries(test_random_map_availability vis map cbba ltl ${LCM_LIBRARIES} ${OpenCV_LIBS})

add_executable(test_threshold_opt test/test_threshold_opt.cpp)
target_link_libraries(test_threshold_opt vis map cbba ltl ${LCM_LIBRARIES} ${OpenCV_LIBS})

