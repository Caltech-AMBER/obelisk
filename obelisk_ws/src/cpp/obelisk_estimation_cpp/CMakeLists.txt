cmake_minimum_required(VERSION 3.8)
project(obelisk_estimation_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Get the Obelisk library
include(FetchContent)
FetchContent_Declare(
  ObeliskCpp
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp/obelisk_cpp
)

FetchContent_MakeAvailable(ObeliskCpp)

# Get Obelisk zoo
include(FetchContent)
FetchContent_Declare(
  Zoo
  SOURCE_DIR $ENV{OBELISK_ROOT}/obelisk/cpp/zoo
)

FetchContent_MakeAvailable(Zoo)

add_executable(jointencoders_passthrough_estimator src/jointencoders_passthrough_estimator.cpp)
target_link_libraries(jointencoders_passthrough_estimator PUBLIC ObeliskCpp Zoo)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  jointencoders_passthrough_estimator
  DESTINATION lib/${PROJECT_NAME})


ament_package()
