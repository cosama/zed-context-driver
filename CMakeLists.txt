CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(zed-context-driver)

SET(EXECUTABLE_OUTPUT_PATH ".")

#find_package(ZED 2 REQUIRED)
#find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(Boost 1.40 COMPONENTS program_options REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
#include_directories(${CUDA_INCLUDE_DIRS})
#include_directories(${ZED_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(src/)

#file(GLOB SOURCES src/*.cpp)
#add_library(${PROJECT_NAME} SHARED ${SOURCES})


#link_directories(${ZED_LIBRARY_DIR})
#link_directories(${CUDA_LIBRARY_DIRS})

add_definitions(-std=c++11 -g -O3)

ADD_EXECUTABLE(zedprocess "main.cpp")

target_link_libraries(zedprocess
                        #${ZED_LIBRARIES}
                        #${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} 
                        ${Boost_LIBRARIES} -lrt -lpthread)

