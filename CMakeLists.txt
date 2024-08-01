cmake_minimum_required(VERSION 3.28)
project(JSLAM2)

set(CMAKE_CXX_STANDARD 17)

# Set paths for external libraries
set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
set(OpenCV_DIR "${EXTERNAL_DIR}/opencv/build")
set(GLEW_DIR "${EXTERNAL_DIR}/glew")
set(GLFW_DIR "${EXTERNAL_DIR}/glfw")
set(EIGEN_DIR "${EXTERNAL_DIR}/eigen")
set(G2O_DIR "${EXTERNAL_DIR}/g2o")
set(G2O_BUILD_DIR "${CMAKE_BINARY_DIR}/g2o-build")

# Find OpenCV
find_package(OpenCV REQUIRED PATHS ${OpenCV_DIR} NO_DEFAULT_PATH)

# Set up GLEW
set(GLEW_INCLUDE_DIR "${GLEW_DIR}/include")
set(GLEW_LIBRARY "${GLEW_DIR}/lib/Release/x64/glew32.lib")

# Set up GLFW
set(GLFW_INCLUDE_DIR "${GLFW_DIR}/include")
set(GLFW_LIBRARY "${GLFW_DIR}/lib-vc2022/glfw3.lib")

# Find OpenGL
find_package(OpenGL REQUIRED)

# Set up Eigen
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "${EIGEN_DIR}")
find_package(Eigen3 3.4 CONFIG QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 CMake configuration not found, using include directory directly.")
    if(EXISTS "${EIGEN_DIR}/Eigen/Core")
        set(EIGEN3_INCLUDE_DIR "${EIGEN_DIR}")
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIR}")
    else()
        message(FATAL_ERROR "Eigen headers not found. Please check the Eigen directory.")
    endif()
else()
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
endif()

# Your main executable
add_executable(JSLAM2 main.cpp)

target_include_directories(JSLAM2 PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${G2O_BUILD_DIR}/include
        ${G2O_DIR}
)

target_link_directories(JSLAM2 PRIVATE
        ${G2O_BUILD_DIR}/lib
)

target_link_libraries(JSLAM2 PRIVATE
        ${OpenCV_LIBS}
        OpenGL::GL
        ${GLEW_LIBRARY}
        ${GLFW_LIBRARY}
        Eigen3::Eigen
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_solver_eigen
        g2o_types_slam3d
        g2o_types_slam2d
        glu32
)

# Copy necessary DLLs to output directory
add_custom_command(TARGET JSLAM2 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OpenCV_DIR}/x64/vc16/bin/opencv_world4100d.dll"
        "${GLEW_DIR}/bin/Release/x64/glew32.dll"
        "${GLFW_DIR}/lib-vc2022/glfw3.dll"
        $<TARGET_FILE_DIR:JSLAM2>
)

# Print debug information
message(STATUS "OpenCV_DIR: ${OpenCV_DIR}")
message(STATUS "OpenCV_VERSION: ${OpenCV_VERSION}")
message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")
message(STATUS "GLEW_INCLUDE_DIR: ${GLEW_INCLUDE_DIR}")
message(STATUS "GLEW_LIBRARY: ${GLEW_LIBRARY}")
message(STATUS "GLFW_INCLUDE_DIR: ${GLFW_INCLUDE_DIR}")
message(STATUS "GLFW_LIBRARY: ${GLFW_LIBRARY}")
message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
message(STATUS "G2O_DIR: ${G2O_DIR}")
message(STATUS "G2O_BUILD_DIR: ${G2O_BUILD_DIR}")