file(GLOB SOURCES ./*.cpp)
file(GLOB HEADERS ./*.h)

add_executable(gp_viewer ${SOURCES} ${HEADERS})
target_link_libraries(gp_viewer pmp csv)

if (EMSCRIPTEN)
    set_target_properties(gp_viewer PROPERTIES LINK_FLAGS "--shell-file ${PROJECT_SOURCE_DIR}/external/pmp/shell.html --preload-file ${PROJECT_SOURCE_DIR}/data@./data")
endif()
