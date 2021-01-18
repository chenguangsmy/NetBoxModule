cmake_minimum_required(VERSION 2.8.0)
project(NetboxModule VERSION 0.1.0)

include(CTest)
enable_testing()

add_executable(NetboxModule main.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
