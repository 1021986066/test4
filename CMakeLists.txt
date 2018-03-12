PROJECT(FindArmor)         

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
# OpenCV > 3 for KCF tracker
#set( CMAKE_EXPORT_COMPILE_COMMANDS ON )
#SET(OpenCV_DIR ${PROJECT_SOURCE_DIR})
FIND_PACKAGE( OpenCV 3 REQUIRED )
# OpenMP
FIND_PACKAGE( OpenMP)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -lfftw3 -lfftw3_omp")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
SET(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
SET(OpenMP_LIBS gomp)
SET(CMAKE_BUILD_TYPE "Debug") 
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
# Declare the target (an executable)
include_directories( ${PROJECT_SOURCE_DIR}/include)
include_directories( ${PROJECT_SOURCE_DIR}/src) 
FILE(GLOB_RECURSE sourcefiles "src/*.cpp")
ADD_EXECUTABLE(FindArmor  
    main.cpp
    ${sourcefiles}
  )
TARGET_COMPILE_OPTIONS(FindArmor PRIVATE ${OpenMP_FLAGS})
TARGET_LINK_LIBRARIES(FindArmor ${OpenCV_LIBS} ${OpenMP_LIBS} /lib/libMVSDK.so)
#MESSAGE(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")     
