# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(PythonLibs 2.7)
find_package(LCM REQUIRED)
find_package(Eigen3 3.1.2)
find_package(Ceres REQUIRED)
include_directories(${CERES_LIBRARIES})


if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})

# Add include directories of third party libraries
include_directories(${CMAKE_SOURCE_DIR}/src/third_party/eigen3)


# Add libraries
set(GPLIB_LIB_SRC 
  src/rbf_kernel.cpp
  src/gaussian_process.cpp
  src/gaussian_process_classification.cpp
  src/log_likelihood_classification.cpp
)

add_library(gplib STATIC ${GPLIB_LIB_SRC})
target_link_libraries(gplib ltl vis utility cbba map auto_vehicle jsoncpp_lib_static)

target_include_directories(gplib PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


add_executable(test_hyperparameters_training test/test_hyperparameters_training.cpp)
target_link_libraries(test_hyperparameters_training gplib ${CERES_LIBRARIES})

add_executable(test_predict_ig test/test_predict_ig.cpp)
target_link_libraries(test_predict_ig gplib cbba ltl vis ${CERES_LIBRARIES} ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(test_gpc test/test_gpc.cpp)
target_link_libraries(test_gpc gplib cbba ltl vis ${CERES_LIBRARIES} ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(test_gpc_sensors test/test_gpc_sensors.cpp)
target_link_libraries(test_gpc_sensors gplib cbba ltl map vis ${CERES_LIBRARIES} ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(test_gpc_ig test/test_gpc_ig.cpp)
target_link_libraries(test_gpc_ig gplib cbba ltl map vis ${CERES_LIBRARIES} ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(test_calculate_ig test/test_calculate_ig.cpp)
target_link_libraries(test_calculate_ig gplib auto_vehicle ltl map vis ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(test_bayesian_simulation test/test_bayesian_simulation.cpp)
target_link_libraries(test_bayesian_simulation gplib auto_vehicle ltl map vis ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(test_bayesian_ipas test/test_bayesian_ipas.cpp)
target_link_libraries(test_bayesian_ipas gplib auto_vehicle ltl map vis jsoncpp_lib_static ${OpenCV_LIBS} ${LCM_LIBRARIES})

add_executable(test_ipas_rSensors test/test_ipas_rSensors.cpp)
target_link_libraries(test_ipas_rSensors gplib auto_vehicle ltl map vis ${OpenCV_LIBS} ${LCM_LIBRARIES})