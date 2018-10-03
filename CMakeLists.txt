project(FindArmor)         

cmake_minimum_required(VERSION 2.6)

# convenient for vscode autocomplete
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# OpenCV > 3 for KCF tracker
find_package(OpenCV 3 REQUIRED )
# message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")     

# OpenMP
find_package(OpenMP)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -pthread")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
set(OpenMP_LIBS gomp)

set(CMAKE_BUILD_TYPE "Debug") 
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

if(CMAKE_BUILD_TYPE STREQUAL Debug)
  message("Debug mode")
  ADD_DEFINITIONS(-D_DEBUG)
endif(CMAKE_BUILD_TYPE STREQUAL Debug)

# Declare the target (an executable)
include_directories( ${PROJECT_SOURCE_DIR}/include)
include_directories( ${PROJECT_SOURCE_DIR}/src) 
file(GLOB_RECURSE sourcefiles "src/*.cpp")
add_executable(FindArmor  
    main.cpp
    ${sourcefiles}
  )
target_compile_options(FindArmor PRIVATE ${OpenMP_FLAGS})
target_link_libraries(FindArmor ${OpenCV_LIBS} ${OpenMP_LIBS} /lib/libMVSDK.so)
