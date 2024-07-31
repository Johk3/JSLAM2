cmake_minimum_required(VERSION 3.28)
project(JSLAM2)

set(CMAKE_CXX_STANDARD 17)

# Set paths for external libraries
set(EXTERNAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external")
set(OpenCV_DIR "${EXTERNAL_DIR}/opencv/build")
set(GLEW_DIR "${EXTERNAL_DIR}/glew")
set(GLFW_DIR "${EXTERNAL_DIR}/glfw")

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

add_executable(JSLAM2 main.cpp)

target_include_directories(JSLAM2 PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${GLEW_INCLUDE_DIR}
        ${GLFW_INCLUDE_DIR}
)

target_link_libraries(JSLAM2 PRIVATE
        ${OpenCV_LIBS}
        OpenGL::GL
        ${GLEW_LIBRARY}
        ${GLFW_LIBRARY}
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