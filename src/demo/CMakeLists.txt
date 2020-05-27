include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(OpenCV REQUIRED)

# Add executables
## demo
## demo
add_executable(vis_map_demo vis_map_demo.cpp)
target_link_libraries(vis_map_demo vis ltl map utility ${OpenCV_LIBS})

add_executable(cbba_demo cbba_demo.cpp)
target_link_libraries(cbba_demo cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(ipas_demo ipas_demo.cpp)
target_link_libraries(ipas_demo cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(info_driven_demo info_driven_demo.cpp)
target_link_libraries(info_driven_demo cbba cbta vis ltl map utility ${OpenCV_LIBS})

add_executable(ipas_bayesian_demo ipas_bayesian_demo.cpp)
target_link_libraries(ipas_bayesian_demo cbba cbta vis ltl map utility ${OpenCV_LIBS})