/***********************************************************************************
 * Header file that defines the different message types passed by shared memory
 * 
 * Author: Marco Salathe <msalathe@lbl.gov>
 * Date:   March 2018
 *
 **********************************************************************************/

#ifndef MSGDEFINITION_HPP
#define MSGDEFINITION_HPP

//largest common factor of 1280x720 and 672x376
//times 4channels (8U-type) per pixel
#define IMAGE_SEND_BITS 768*4

//time stamp format similar to C's timeval
struct Timestamp{
  long long sec, usec;
};

//image header format: the length of the header 
//defines the block size of an image, the actual
//image data needs to be dividable by this length
//the padding is added to inflate the header to
//the correct size, this adds ~5MB for 900msgs,
//the header size is the image size in bytes, the
//type the opencv mat type and block_size and nmb
//define the size and nmb of blocks in a given image
struct ImageHeaderMsg {
  Timestamp time;
  int size;
  int cols;
  int rows;
  int type;
  int block_size;
  int block_nmb;
  char pad[IMAGE_SEND_BITS-24-sizeof(Timestamp)];
};

//pose message format
struct PoseMsg {
  Timestamp time;
  float position[3];
  float orientation[4];
};

//imu message format
struct ImuMsg {
  Timestamp time;
  float orientation[4];
  float angular_velocity[3];
  float linear_acceleration[3];
};

//Camera information structure
//see http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
//all matrices are row major matrices
struct CameraInfo {
  //image size in pixels
  float height, width;

  //plumb_bob distortion model
  //D = [k1, k2, t1, t2, k3]
  float D[5];

  //intrinsic camera matrix for raw images
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  float K[9];

  //rectification matrix (stereo cameras only)
  //A rotation matrix aligning the camera coordinate system to the ideal
  //stereo image plane so that epipolar lines in both stereo images are
  //parallel.
  float R[9];

  //Projection/camera matrix
  //the intrinsic (camera) matrix of the processed (rectified) image
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  float P[12];

  //rectified or not
  bool rectified;
};

#endif //#ifndef MSGDEFINITION_HPP
