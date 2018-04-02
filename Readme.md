
# ZED Context Driver

## Introduction

This is a simple implementation of a program that controls a ZED/ZEDMini Camera through the SDK and stores information from the Camera to a shared buffer, accessible by other threads/processes running on the same computer.

It contains two header files **ZEDWrapper.hpp** and **SharedBuffer.hpp** (located in the src-folder), and some simple programs to run/show the drivers ability.

### SharedBuffer.hpp

A class template that uses boost to implement shared memory buffers. It deals with a lot of the annoyances that come with using shared memory and offers a simple framework to use these buffers.

### ZEDWrapper.hpp

A standard class that keeps track of all the ZED Camera parameters and starts a thread to control the Camera with the SDK. This requires to create a ZEDWrapper object, define all the parameters with the setter methods and then call the startWrapperThread() member function to start the thread. The thread is stopped when the destructor of the ZEDWrapper is called. Some parameters, such as rates can be updated on the fly. Also the mesh can be saved any time with the saveMesh() member function.

### zeddriver.cpp

A small script that creates a ZEDWrapper class and allows to create and control the wrapper class from the command line.

Use it by calling for example: `./zeddriver --tracking MySharedBufferOdometryTopicName --left MySharedBufferLeftImageTopicName --right MySharedBufferRightImageTopicName --length 100`

For more details see the script itself.

### zedtester.cpp

A script that can be used to read information stored by zeddriver.cpp to shared buffers. Use it by calling:

* Display image data from a shared buffer: `zedtester MySharedBufferOdometryTopicName image`

* Printing odometry data to the terminal (stdout): `zedtester MySharedBufferOdometryTopicName odom`

* Creating an image buffer and read images from the zed camera to it: `zedtester MySharedBufferImageTopicName`

## Installation

The driver uses CMake. Just create a folder build, cd to it and call `cmake .. && make`
