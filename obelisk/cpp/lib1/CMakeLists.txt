message(STATUS "Building lib1")

add_library(lib1
    place_holder.cpp
    include/place_holder.h)

target_include_directories(lib1 PUBLIC include/)
