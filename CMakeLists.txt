cmake_minimum_required(VERSION 2.4)
project(zed-context-driver)

option(LOCAL_OPENCV "LOCAL_OPENCV" OFF)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 OLD)
  cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

set(EXECUTABLE_OUTPUT_PATH ".")

if(LOCAL_OPENCV)
  find_package(OpenCV REQUIRED PATHS ${CMAKE_CURRENT_SOURCE_DIR}/opencv-static NO_DEFAULT_PATH)
  message(STATUS "Found local OpenCV ${OpenCV_VERSION} in ${OpenCV_INSTALL_PATH}")
else()
  find_package(OpenCV REQUIRED)  
  message(STATUS "Found global OpenCV ${OpenCV_VERSION} in ${OpenCV_INSTALL_PATH}")
endif(LOCAL_OPENCV)

find_package(ZED 2 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(Boost 1.40 REQUIRED)

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIR})

add_library(zedwrapper SHARED ${CMAKE_CURRENT_SOURCE_DIR}/include/ZEDWrapperPublic.cpp)

set(LIBRARIES ${ZED_LIBRARIES}
              ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} 
              ${OpenCV_LIBRARIES} -lrt -lpthread)

add_definitions(-std=c++11 -O3) # -DUSE_MANAGED_SHARED_MEMORY)

add_executable(zedtester "zedtester.cpp")
add_executable(zeddriver "zeddriver.cpp")
add_executable(buffertest "buffertest.cpp")
add_executable(imagetest "imagetest.cpp")

target_link_libraries(zedwrapper ${LIBRARIES})
target_link_libraries(zedtester ${LIBRARIES})
target_link_libraries(zeddriver ${LIBRARIES})
target_link_libraries(buffertest -lrt -lpthread)
target_link_libraries(imagetest ${OpenCV_LIBRARIES} -lrt -lpthread)

set(directory delivery)
add_custom_target(delivery
    COMMAND ${CMAKE_COMMAND} -E make_directory ${directory} DEPENDS delivery_libs delivery_incs)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/delivery/zeddriver.cpp ${CMAKE_CURRENT_BINARY_DIR}/${directory}/zeddriver.cpp)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/zedtester ${CMAKE_CURRENT_BINARY_DIR}/${directory}/zedtester)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/delivery/CMakeLists.txt ${CMAKE_CURRENT_BINARY_DIR}/${directory}/CMakeLists.txt)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/delivery/Readme.md ${CMAKE_CURRENT_BINARY_DIR}/${directory}/Readme.md)

add_custom_target(delivery_libs
    COMMAND ${CMAKE_COMMAND} -E make_directory ${directory}/lib)
add_custom_command(
        TARGET delivery_libs POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/libzedwrapper.so ${CMAKE_CURRENT_BINARY_DIR}/${directory}/lib/libzedwrapper.so)
add_custom_command(
        TARGET delivery_libs POST_BUILD COMMAND strip --strip-all ${CMAKE_CURRENT_BINARY_DIR}/${directory}/lib/libzedwrapper.so)

add_custom_target(delivery_incs
    COMMAND ${CMAKE_COMMAND} -E make_directory ${directory}/include)
add_custom_command(
        TARGET delivery_incs POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/ZEDWrapperPublic.hpp ${CMAKE_CURRENT_BINARY_DIR}/${directory}/include/ZEDWrapperPublic.hpp)
add_custom_command(
        TARGET delivery_incs POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_CURRENT_SOURCE_DIR}/include/ZEDMsgDefinition.hpp ${CMAKE_CURRENT_BINARY_DIR}/${directory}/include/ZEDMsgDefinition.hpp)
