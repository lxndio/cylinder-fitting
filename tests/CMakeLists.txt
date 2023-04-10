file(GLOB TESTS "./*.cpp")

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.3.2
)
FetchContent_MakeAvailable(Catch2)

add_executable(Ransac_test ${TESTS})
target_link_libraries(Ransac_test PRIVATE Catch2::Catch2WithMain ransac_lib)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(Ransac_test)
