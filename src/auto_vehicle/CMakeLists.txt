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
set(AUTOVEHICLE_LIB_SRC
    src/auto_vehicle.cpp  
    src/tasks.cpp
)

add_library(auto_vehicle STATIC ${AUTOVEHICLE_LIB_SRC})
target_link_libraries(auto_vehicle ltl vis utility map ${LCM_LIBRARIES})

target_include_directories(auto_vehicle PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

add_executable(test_cbba_newlib test/test_cbba_newlib.cpp)
target_link_libraries(test_cbba_newlib vis auto_vehicle map utility ${OpenCV_LIBS})

add_executable(test_ipas_decentralized test/test_ipas_decentralized.cpp)
target_link_libraries(test_ipas_decentralized vis auto_vehicle map utility ${LCM_LIBRARIES})

add_executable(test_info_driven test/test_info_driven.cpp)
target_link_libraries(test_info_driven vis auto_vehicle map utility ${LCM_LIBRARIES})

#add_executable(test_optimality_analysis test/test_optimality_analysis.cpp)
#target_link_libraries(test_optimality_analysis vis auto_vehicle map utility ${LCM_LIBRARIES})

#add_executable(test_threshold_analysis test/test_threshold_analysis.cpp)
#target_link_libraries(test_threshold_analysis vis auto_vehicle map utility ${LCM_LIBRARIES})