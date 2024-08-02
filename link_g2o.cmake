# Find g2o libraries
find_library(G2O_CORE_LIBRARY g2o_core PATHS "${G2O_LIB_DIR}" NO_DEFAULT_PATH)
find_library(G2O_STUFF_LIBRARY g2o_stuff PATHS "${G2O_LIB_DIR}" NO_DEFAULT_PATH)
find_library(G2O_TYPES_SBA_LIBRARY g2o_types_sba PATHS "${G2O_LIB_DIR}" NO_DEFAULT_PATH)
find_library(G2O_SOLVER_EIGEN_LIBRARY g2o_solver_eigen PATHS "${G2O_LIB_DIR}" NO_DEFAULT_PATH)
find_library(G2O_TYPES_SLAM3D_LIBRARY g2o_types_slam3d PATHS "${G2O_LIB_DIR}" NO_DEFAULT_PATH)
find_library(G2O_TYPES_SLAM2D_LIBRARY g2o_types_slam2d PATHS "${G2O_LIB_DIR}" NO_DEFAULT_PATH)

# Check if all libraries were found
if(NOT G2O_CORE_LIBRARY OR
        NOT G2O_STUFF_LIBRARY OR
        NOT G2O_TYPES_SBA_LIBRARY OR
        NOT G2O_SOLVER_EIGEN_LIBRARY OR
        NOT G2O_TYPES_SLAM3D_LIBRARY OR
        NOT G2O_TYPES_SLAM2D_LIBRARY)
    message(FATAL_ERROR "One or more g2o libraries not found")
endif()

# Link the libraries
execute_process(
        COMMAND lib.exe /OUT:${TARGET_FILE} ${TARGET_FILE}
        ${G2O_CORE_LIBRARY}
        ${G2O_STUFF_LIBRARY}
        ${G2O_TYPES_SBA_LIBRARY}
        ${G2O_SOLVER_EIGEN_LIBRARY}
        ${G2O_TYPES_SLAM3D_LIBRARY}
        ${G2O_TYPES_SLAM2D_LIBRARY}
        RESULT_VARIABLE link_result
)

if(NOT link_result EQUAL 0)
    message(FATAL_ERROR "Failed to link g2o libraries")
endif()

message(STATUS "Successfully linked g2o libraries")