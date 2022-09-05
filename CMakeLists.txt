##############################################################################
# paths
##############################################################################

set(PROJECT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})


##############################################################################
# dependencies
##############################################################################

# dependencies
cmake_policy(SET CMP0072 NEW)
find_package(OpenGL REQUIRED)


##############################################################################
# compiler flags
##############################################################################

# we need C++11
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Windows needs special treatment
if(WIN32)
  add_compile_definitions(_USE_MATH_DEFINES NOMINMAX _CRT_SECURE_NO_WARNINGS)
  add_compile_options(/wd4267) # conversion from size_t
  add_compile_options(/wd4244) # conversion from double to float
  add_compile_options(/wd4305) # truncation from double to float
  add_compile_options(/wd4005) # macro redefinition
endif()

# emscripten is used to compile web-demos
if (EMSCRIPTEN)
  add_compile_options(--no-heap-copy)
  add_link_options("SHELL:-s WASM=1 -s USE_WEBGL2=1 -s USE_GLFW=3 -s ALLOW_MEMORY_GROWTH=1 -s MINIFY_HTML=0")
  set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()


##############################################################################
# build type
##############################################################################

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb")
endif()


##############################################################################
# STB 
# (place *before* GLFW since GLFW has an old copy of stb_image_write.h)
##############################################################################

set(STB_SOURCE_DIR "external/stb_image")
include_directories(${STB_SOURCE_DIR})
add_subdirectory(${STB_SOURCE_DIR})

##############################################################################
# GLFW
##############################################################################

if(NOT EMSCRIPTEN)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "")
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "")
  set(GLFW_INSTALL OFF CACHE BOOL "")
  add_subdirectory(external/glfw)
  include_directories(external/glfw/include
                      external/glfw/deps)
endif()

##############################################################################
# RPLY 
##############################################################################

set(RPLY_SOURCE_DIR "external/rply")
include_directories(${RPLY_SOURCE_DIR})
add_subdirectory(${RPLY_SOURCE_DIR})

##############################################################################
# GLEW
##############################################################################

if(NOT EMSCRIPTEN)
  include_directories(external/glew/include)
  add_definitions(-DGLEW_STATIC)
  add_library(glew STATIC
              external/glew/src/glew.c
              external/glew/include)
  target_link_libraries(glew ${GLFW_LIBRARIES})
endif()

##############################################################################
# imgui
##############################################################################

set(IMGUI_SOURCE_DIR "external/imgui")
include_directories(${IMGUI_SOURCE_DIR})
add_subdirectory(${IMGUI_SOURCE_DIR})


##############################################################################
# Eigen
##############################################################################

set(EIGEN_SOURCE_DIR "external/eigen")
include_directories(${EIGEN_SOURCE_DIR})


##############################################################################
# pmp
##############################################################################

set(PMP_SOURCE_DIR "external/pmp")
include_directories(external)
add_subdirectory(external/pmp)


##############################################################################
# csv-parser
##############################################################################

add_subdirectory(external/csv-parser)


##############################################################################
# source code directories
##############################################################################

if (EMSCRIPTEN)
    add_definitions(-DPOINTSET_DIRECTORY="./data/pointsets/")
else()
    add_definitions(-DPOINTSET_DIRECTORY="${CMAKE_SOURCE_DIR}/data/pointsets/")
endif()

include(AddFileDependencies)
include_directories(${PROJECT_SOURCE_DIR}/src/)
add_subdirectory(src)



##############################################################################
