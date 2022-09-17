file(GLOB_RECURSE SRCS ./*.cpp)
file(GLOB_RECURSE HDRS ./*.h)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost REQUIRED)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIR})
endif()

add_library(utilities STATIC ${SRCS} ${HDRS})
