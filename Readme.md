
# ZED Context Driver

## Introduction

This is a simple implementation of a program that controls a ZED/ZEDMini Camera through the SDK and stores information from the Camera to a shared buffer, accessible by other threads/processes running on the same computer.

It contains three header files **ZEDWrapper.hpp**, **MsgDefinition.hpp** and **SharedBuffer.hpp**  (located in the src-folder), and some simple programs to run/show the drivers ability.

### SharedBuffer.hpp

A class template that uses boost to implement shared memory buffers. It deals with a lot of the annoyances that come with using shared memory and offers a simple framework to use these buffers.

### MsgDefinition.hpp

Definition of the msg format that is used in the shared buffer (such as pose, imu, images, camera infos)

### ZEDWrapper.hpp

A standard class that keeps track of all the ZED Camera parameters and starts a thread to control the Camera with the SDK. This requires to create a ZEDWrapper object, define all the parameters with the setter methods and then call the startWrapperThread() member function to start the thread. The thread is stopped when the destructor of the ZEDWrapper is called. Some parameters, such as rates can be updated on the fly. Also the mesh can be saved any time with the saveMesh() member function.


### zeddriver.cpp

A small script that creates a ZEDWrapper class and allows to create and control the wrapper class from the command line.

Use it by calling for example: `./zeddriver --frame-rate 30 --left MySharedBufferLeftImageTopicName --right MySharedBufferRightImageTopicName --length 100` to store images to a memory buffer (30Hz) or `./zeddriver --local ./ 1 jpg` to store images as jpg's at 1Hz.

For more details see the script itself.

### zedtester.cpp

A script that can be used to read information stored by zeddriver.cpp to shared buffers. Use it by calling:

* Display image data from a shared buffer: `zedtester MySharedBufferOdometryTopicName image`

* Printing odometry data to the terminal (stdout): `zedtester MySharedBufferOdometryTopicName odom`

* Creating an image buffer and read images from the zed camera to it: `zedtester MySharedBufferImageTopicName`

## Installation

The driver uses CMake. Just create a folder build, cd to it and call `cmake .. && make`

## Dependencies

* ZED SDK (CUDA8 or 9)
* OpenCV
* Boost

## Copyright Notice

Zed Context Driver, Copyright (c) 2018, The Regents of the University of California, through Lawrence Berkeley National Laboratory (subject to receipt of any required approvals from the U.S. Dept. of Energy).  All rights reserved.

If you have questions about your rights to use or distribute this software, please contact Berkeley Lab's Intellectual Property Office at  IPO@lbl.gov.

NOTICE.  This Software was developed under funding from the U.S. Department of Energy and the U.S. Government consequently retains certain rights. As such, the U.S. Government has been granted for itself and others acting on its behalf a paid-up, nonexclusive, irrevocable, worldwide license in the Software to reproduce, distribute copies to the public, prepare derivative works, and perform publicly and display publicly, and to permit other to do so.

## License Agreement

"Zed Context Driver" Copyright (c) 2018, The Regents of the University of California, through Lawrence Berkeley National Laboratory (subject to receipt of any required approvals from the U.S. Dept. of Energy).  All rights reserved."

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

(1) Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

(2) Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

(3) Neither the name of the University of California, Lawrence Berkeley National Laboratory, U.S. Dept. of Energy nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER, U.S. DEPT. OF ENERGY, NOR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

You are under no obligation whatsoever to provide any bug fixes, patches, or upgrades to the features, functionality or performance of the source code ("Enhancements") to anyone; however, if you choose to make your Enhancements available either publicly, or directly to Lawrence Berkeley National Laboratory, without imposing a separate written license agreement for such Enhancements, then you hereby grant the following license: a  non-exclusive, royalty-free perpetual license to install, use, modify, prepare derivative works, incorporate into other computer software, distribute, and sublicense such enhancements or derivative works thereof, in binary and source code form.
