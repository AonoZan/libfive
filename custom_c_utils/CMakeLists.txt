cmake_minimum_required(VERSION 3.12)
project(custom_c_utils)

add_library(custom_c_utils SHARED custom_c_utils.cpp)

set_target_properties(custom_c_utils PROPERTIES
    INSTALL_RPATH "$ORIGIN")

# Link to the main libfive shared library to resolve symbols at runtime
target_link_libraries(custom_c_utils PRIVATE libfive)

if(UNIX AND NOT APPLE)
    target_link_libraries(custom_c_utils PRIVATE Threads::Threads)
endif()

install(TARGETS custom_c_utils DESTINATION lib)