# Include path
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)
find_package(MathGL2 2.4)
find_package(PythonLibs 2.7)
find_package(LCM REQUIRED)


if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})


# Add libraries
set(VIS_LIB_SRC
    src/vis_utils.cpp
    src/graph_vis.cpp
    src/graph_vis_lcm.cpp
)

add_library(vis STATIC ${VIS_LIB_SRC})
target_link_libraries(vis map utility cbba ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
target_include_directories(vis PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)


include_directories(
    ${MathGL_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

#add_executable(test_uncertain_map_vis test/test_uncertain_map_vis.cpp)
#target_link_libraries(test_uncertain_map_vis vis map cbba ${LCM_LIBRARIES})



