cmake_minimum_required(VERSION 2.4)
project(zed-context-driver)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

set(EXECUTABLE_OUTPUT_PATH ".")

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

add_executable(zedtester "zedtester.cpp")
add_executable(zeddriver "zeddriver.cpp")
add_executable(buffertest "buffertest.cpp")
add_executable(imagetest "imagetest.cpp")

add_library(zedwrapper SHARED src/ZEDWrapperPublic.cpp)

target_link_libraries(zedtester
                         ${ZED_LIBRARIES}
                         ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} 
                         ${OpenCV_LIBRARIES} -lrt -lpthread)

target_link_libraries(zeddriver
                         zedwrapper ${ZED_LIBRARIES}
                         ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY} ${CUDA_NPP_LIBRARIES_ZED} 
                         ${OpenCV_LIBRARIES} -lrt -lpthread)

target_link_libraries(buffertest -lrt -lpthread)

target_link_libraries(imagetest
                         ${OpenCV_LIBRARIES} -lrt -lpthread)


#set(directory delivery)
#set(CMAKE_INSTALL_PREFIX ${delivery})
#install(DIRECTORY DESTINATION ${directory})
#install(TARGETS zedwrapper DESTINATION ${CMAKE_INSTALL_PREFIX} EXPORT myproj-targets)
#install(EXPORT myproj-targets DESTINATION ${CMAKE_INSTALL_PREFIX})




set(directory delivery)
add_custom_target(delivery ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory ${directory})
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ZEDWrapperPublic.hpp ${CMAKE_CURRENT_BINARY_DIR}/${directory}/ZEDWrapperPublic.hpp)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ZEDMsgDefinition.hpp ${CMAKE_CURRENT_BINARY_DIR}/${directory}/ZEDMsgDefinition.hpp)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/delivery/zeddriver.cpp ${CMAKE_CURRENT_BINARY_DIR}/${directory}/zeddriver.cpp)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/libzedwrapper.so ${CMAKE_CURRENT_BINARY_DIR}/${directory}/libzedwrapper.so)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_BINARY_DIR}/zedtester ${CMAKE_CURRENT_BINARY_DIR}/${directory}/zedtester)
add_custom_command(
        TARGET delivery POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy
        ${CMAKE_CURRENT_SOURCE_DIR}/delivery/CMakeLists.txt ${CMAKE_CURRENT_BINARY_DIR}/${directory}/CMakeLists.txt)
