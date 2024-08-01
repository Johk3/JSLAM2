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
    set(EIGEN3_INCLUDE_DIR "${EIGEN_DIR}")
    if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
        message(FATAL_ERROR "Eigen headers not found. Please check the Eigen directory.")
    endif()
else()
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
endif()

# Set up g2o
set(G2O_BUILD_APPS OFF CACHE BOOL "Build g2o apps")
set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "Build g2o examples")
set(G2O_USE_OPENGL OFF CACHE BOOL "Build g2o with OpenGL support")
set(G2O_BUILD_SHARED_LIBS OFF CACHE BOOL "Build g2o shared libraries")
set(G2O_LGPL_SHARED_LIBS OFF CACHE BOOL "Build g2o LGPL shared libs")

# Add g2o cmake modules to module path
list(APPEND CMAKE_MODULE_PATH ${G2O_DIR}/cmake_modules)

# Add g2o as subdirectory
add_subdirectory(${G2O_DIR} ${CMAKE_BINARY_DIR}/g2o)

# Your main executable
add_executable(JSLAM2 main.cpp)

target_include_directories(JSLAM2 PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
        ${G2O_DIR}
        ${G2O_DIR}/g2o
)

# List g2o libraries
set(G2O_LIBS
        g2o_core
        g2o_stuff
        g2o_types_sba
        g2o_solver_eigen
        g2o_types_slam3d
        g2o_types_slam2d
)

target_link_libraries(JSLAM2 PRIVATE
        ${OpenCV_LIBS}
        OpenGL::GL
        ${GLEW_LIBRARY}
        ${GLFW_LIBRARY}
        $<$<BOOL:${Eigen3_FOUND}>:Eigen3::Eigen>
        ${G2O_LIBS}
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

