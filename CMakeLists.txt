CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
PROJECT(zed-context-driver)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH ".")

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(Boost 1.40 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(src/)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIR})

add_definitions(-std=c++11 -g -O3)

ADD_EXECUTABLE(zedtester "zedtester.cpp")
ADD_EXECUTABLE(zeddriver "zeddriver.cpp")
ADD_EXECUTABLE(buffertest "buffertest.cpp")
ADD_EXECUTABLE(imagetest "imagetest.cpp")


target_link_libraries(zedtester
                         ${ZED_LIBRARIES}
                         ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} 
                         ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} -lrt -lpthread)

target_link_libraries(zeddriver
                         ${ZED_LIBRARIES}
                         ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} 
                         ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} -lrt -lpthread)

target_link_libraries(buffertest
                        ${Boost_LIBRARIES} -lrt -lpthread)

target_link_libraries(imagetest
                         ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} -lrt -lpthread)

