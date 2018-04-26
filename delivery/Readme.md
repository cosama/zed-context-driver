## Requirements

* Ubuntu 16.04
 (best installed with Jetpack 3.2 (https://developer.nvidia.com/embedded/jetpack)
* CUDA-9.0 (also best installed with Jetpack 3.2)
* ZED SDK v2.4 for TX2 (https://download.stereolabs.com/zedsdk/2.4/tegrax2)
* cmake (`sudo apt-get install cmake`)

## Compiling

cd to the folder in a terminal and call `mkdir build && cd build && cmake .. && make`

## Detail on included scripts / program

### lib/libzedwrapper.so include/ZEDWrapperPublic.hpp

The shared library and the respective header file to access it.

### zeddriver.cpp

A small script that creates a ZEDWrapperPublic class and allows to create and control the wrapper class from the command line.

Use it by calling for example: 

* `./zeddriver --frame-rate 30 --left MySharedBufferLeftImageTopicName --right MySharedBufferRightImageTopicName --length 100` to store images to a memory buffer (30Hz).
* `./zeddriver --tracking MySharedBufferOdometryTopicName --imu MySharedBufferImuTopicName` to perform tracking and store the odometry data and imu data to a memory buffer.
* `./zeddriver --local ./myfolder 1 jpg` to store images as jpg's at 1Hz to myfolder.

Any of the options can be passed independently of each other or together. For more details see the script itself.

### ./zedtester

A precompiled script that can be used to read information stored by libzedwrapper to shared buffers. Use it by calling:

* Display image data from a shared buffer topic: `./zedtester MySharedBufferOdometryTopicName image`

* Printing odometry data to the terminal (stdout): `./zedtester MySharedBufferOdometryTopicName odom`

* Creating an image buffer and read images from the zed camera to it: `./zedtester MySharedBufferImageTopicName`


## Errors

* The shared buffer require locking on write/reading, if a program dies while it is writing/reading to/from the buffer it might lock the buffer indefinitely. The easiest solution to resolve this situation is to remove the buffer altogether and free up the resources again. In Linux the buffer is stored at /dev/shm/MySharedBufferTopicName and can be removed with root access and 'rm'.

* After rebooting the TX2 the ZED-SDK (libzedwrapper) reports 'CAMERA NOT DETECTED'. This is a bug with older firmware and should be fixed with the most recent version (comes with the ZED SDK v2.4, but needs to be flashed to the camera). To fix this issue, just unplug and reconnect the ZEDMini camera.
