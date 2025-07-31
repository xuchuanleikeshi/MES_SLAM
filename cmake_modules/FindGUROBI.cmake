# ------------------------------------------------------------------------------
# This file sets up Gurobi for CMake. Once done this will define
#
#   GUROBI_FOUND           - system has GUROBI
#   GUROBI_INCLUDE_DIRS    - the GUROBI include directories
#   GUROBI_LIBRARIES       - Link these to use GUROBI
#
#  In your CMakeLists file, you need to add, e.g. (modify it if necessary):
#        if (GUROBI_FOUND)
#            message(STATUS "Gurobi include dir: " ${GUROBI_INCLUDE_DIRS})
#            message(STATUS "Gurobi libraries: " ${GUROBI_LIBRARIES})
#            target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_GUROBI)
#            target_include_directories(${PROJECT_NAME} PRIVATE ${GUROBI_INCLUDE_DIRS})
#            target_link_libraries(${PROJECT_NAME} PRIVATE ${GUROBI_LIBRARIES})
#        endif()
# ------------------------------------------------------------------------------


# Is it already configured?
if(NOT GUROBI_FOUND)
    if(NOT DEFINED GUROBI_HOME)
        set(GUROBI_HOME "/opt/gurobi1201/linux64")  # 设默认路径
    endif()

    find_path(GUROBI_INCLUDE_DIR gurobi_c++.h
            HINTS ${GUROBI_HOME}/include
    )

    find_library(GUROBI_LIBRARY
            NAMES gurobi gurobi120 gurobi110
            HINTS ${GUROBI_HOME}/lib
    )

    find_library(GUROBI_CXX_LIBRARY
            NAMES gurobi_c++
            HINTS ${GUROBI_HOME}/lib
    )

    set(GUROBI_INCLUDE_DIRS ${GUROBI_INCLUDE_DIR})
    set(GUROBI_LIBRARIES ${GUROBI_CXX_LIBRARY} ${GUROBI_LIBRARY})

endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI REQUIRED_VARS GUROBI_INCLUDE_DIRS GUROBI_LIBRARIES)

mark_as_advanced(GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_INCLUDE_DIR)
