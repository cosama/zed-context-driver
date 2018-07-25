/***********************************************************************************
 * Header file that defines the different message types passed by shared memory
 * 
 * Author:  Marco Salathe <msalathe@lbl.gov>
 * Date:    March 2018
 * License: See License.txt in parent folder of this repository.
 *
 **********************************************************************************/

#ifndef MSGDEFINITION_HPP
#define MSGDEFINITION_HPP

//largest common factor of 1280x720 and 672x376 times 4channels (8U-type) per pixel
#define IMAGE_SEND_BITS 768*4

#define MSG_STRING_LENGTH 128

//time stamp format similar to C's timeval, seconds and microseconds
//if thrift provides timestamps only in millsec, just convert them
//to the appropriate number, however tracking benefits from higher 
//precision
struct Timestamp{
  long long sec, usec;
};

//pose message format
struct PoseMsg {
  Timestamp time;
  float position[3];
  float orientation[4];
  float confidence;
};

//imu message format
struct ImuMsg {
  Timestamp time;
  float orientation[4];  
  float angular_velocity[3];
  float linear_acceleration[3];
};

//Thrift-like CameraIntrinsics
struct CameraIntrinsics {
  double cx, cy; //horizontal, vertical optical center in pixel #'s
  double fx, fy; //x/y vocal length in pixel #'s
  double K[3];   //radial distortion coefficients
  double P[2];   //tangential distortion coefficients
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

//image header format: the length of the header defines the block size of an image, 
//the actual image data needs to be dividable by this length. The padding (pad) is added 
//to inflate the header to the correct size, this adds ~5MB for 900msgs. 
//Below, 'size' is the image size in bytes, 'type' is the opencv mat type and 'block_size' and 'block_nmb'
//define the size and # of blocks in a given image. Optimally the 'data' pointer
//defines the memory block (continous) where the image is stored. Can be converted by: 
// cv::Mat(header->rows, header->cols, header->type, header->data);
struct ImageHeaderMsg {

  //Image and data organization information (slightly redundant)
  int size; // in bytes
  int cols; // image dimensions
  int rows; // image dimensions
  int type; // opencv image type
  int block_size; // equal to IMAGE_SEND_BITS
  //Block size must be > 6*sizeof(int)+sizeof(Timestamp)+sizeof(short)+11*sizeof(double)+sizeof(void*)
  int block_nmb; //number of data blocks of block_size length

  // Timestamp
  Timestamp time;

  // Image meta data, useful if image needs to be displayed on map or if it is an overlay
  short resultType; //enum as in thrift, 0: ImageOverlay, 1 MapOverlay, and -1 for someting else
  double location[3]; // 0: latitude, 1: longitude, 2: altitude in degrees  (top-left corner according to Thrift Def)
  double orientation[4]; // quaternion describing orientation of PTU when image was taken (with camera orientation system def, defines camera orientation)
  double width, height; // width and heights in meters Required for MapOverlay
  double horizontalFOV, verticalFOV; //in degrees Required for Image overlay

  // Misc stuff required for shared buffer implmentation
  void *data; //pointer to image, not usable in received data, but can be replaced with actual value
  // header pad to ensure header size is equal to block size.  
  char pad[IMAGE_SEND_BITS-6*sizeof(int)-sizeof(Timestamp)-sizeof(short)-11*sizeof(double)-sizeof(void*)];
};

//GPS message: the format is general, but fields marked with required are needed for 
//the LBL-algorithm to work, all the others are optional, but might help in some cases.
//The respective entries are according to the thrift definition of sensorFrameOfReference.
struct GpsMsg {
  // Header data, general information about the fields
  Timestamp timeStamp;
  short sensorFrameOfReference;  // same as thrift enum, best Geodetic_NED
  short datum;  // same as thrift enum, best WGS84
  short health; // same as thrift enum

  // Booleans to pass information about presence of fields
  bool hasLocation[3];
  bool hasPosition[3];
  bool hasAcceleration[3];
  bool hasOrientation[3];
  bool hasVelocity[3];
  bool hasSpeed;
  bool hasAngularRate[3];
  bool hasGravityCompAccel[3];
  bool hasPositionErrorXYZ[3];
  bool hasPositionError;
  bool hasOrientationError[3];
  bool hasTimeError;

  // GPS information (required)
  short numberOfSatellites; // same as thrift value or -1 if not provided
  short qualityOfFix; // same as thrift enum or -1 if not provided

  // Position (one or the other is required, prefered is location)
  double location[3]; // 0: latitude, 1: longitude, 2: altitude in degrees
  double position[3]; // 0: x, 1: y, 2: z in meters

 // Acceleration (required)
  double acceleration[3]; // 0: x, 1: y, 2: z in m/s%2

  // Orientation (required)
  double orientation[3]; // 0: yaw, 1: pitch, 2: roll in degrees

  // Velocities (if available, not necessary) 
  double velocity[3]; // meters per second
  double speed;       // meters per second

  // Angular rate (required)
  double angularRate[3]; // 0: x, 1: y, 2: z in degrees per second, where xyz defines righthanded rotation around that axis

  // Gravity compensated acceleration (if available, not necessary) 
  double gravityCompAccel[3]; // 0: x, 1: y, 2: z in m/s%2

  // Estimated position error (one or the other is required)
  double positionErrorXYZ[3]; // 0: x, 1: y, 2: z in meters
  double positionError; // in meters

  // Estimated orientation error (required)
  double orientationError[3]; // 0: yaw, 1: pitch, 2: roll in degrees

  // Estimated time error (if available, not necessary) 
  long long timeError; // in nanoseconds

  // Timestamp of reference frame, required for LocalXYZ frame, otherwise empty
  Timestamp referenceFrameTimeStamp; // needs
};

struct SBEOutputMsg{
	// PSI fill in content of SBE message.  We assume this will be a subset of WIND Thrift Localization outupt structure.
	// We also don't care if the 
	/* Also we should consider whether we should expect 1 localization output or a multitude of locations.  Including:
		Same "Event" different time
		Same "Event" different analysis states -- e.g. based on LBL odometry vs. based on raw GPS odometry
		*/
};

// Configuration and status message. Informs algorithm about 
// crucial variables and settings, can be sent very sparsely,
// but must be sent on status changes (like alarm was raised
// or stopped).
struct AlgorithmConfigurationMsg {
  //Time stamp of message
  Timestamp timeStamp;

  // Alarm
  bool inAlarm;

  // powerMode -- 1 'Standard' supported presently -- 0 'LowPower', 2 'HighPower' and 3 'Auto' to be supported.
  short powerMode; //powerMode of tracking needs to be adjusted on the camera side, this only controls powerMode of algorithm

  // Information about Zed Shared Buffer (if provided, topic will be enabled otherwise set to "")
  // Should be sent from Leidos over thrift
  char poseFileName[MSG_STRING_LENGTH]; //shared buffer name of tracking/odometry topic (required)
  char leftImageFileName[MSG_STRING_LENGTH]; // stereo camera (left) image (required)
  char rightImageFileName[MSG_STRING_LENGTH];// stereo camera (right) image (required)
  char cameraImuFileName[MSG_STRING_LENGTH]; // ZEDMini IMU topic name (not required)
  char depthImageFileName[MSG_STRING_LENGTH]; // depth image topic name (not required for Standard powerMode)

  // Location of Camera and Navigation Sensor
  PoseMsg cameraPose; // position and orientation of camera with respect to PTU frame
  PoseMsg navigationPose; // position and orientation of navigation sensor (VectorNav) with respect to PTU frame (usually no translation/orientation)

  // Camera intrinsics
  CameraIntrinsics intrinsics; // ZEDMini intrinsics, received from thrift

  // PSI to LBL communication Shared Buffer Names
  char navigationGPSRawFileName[MSG_STRING_LENGTH]; // name of shared buffer where GpsMsg is sent from PSI to LBL
  char localizationSBEOutput[MSG_STRING_LENGTH]; // name of shared buffer where SBEOutputMsg struct will be sent from PSI to LBL during alarm

  // LBL to PSI communication Shared Buffer Names
  char navigationGPSSolutionFileName[MSG_STRING_LENGTH]; // name of shared buffer where GpsMsg is sent from LBL to PSI
  char imageOverlayFileName[MSG_STRING_LENGTH]; // name of shared buffer where ImageHeaderMsg (for ImageOverlay) is sent from LBL to PSI
  char mapOverlayFileName[MSG_STRING_LENGTH]; // name of shared buffer where ImageHeaderMsg (for MapOverlay) is sent from LBL to PSI

  // Configure LBL algorithm and outputs  
  bool startProvided, stopProvided; // if custom time range is provided, otherwise take earliest possible and halt when inAlarm=0 (see ICD doc)
  Timestamp startStamp; // all information before that time will be ignored
  Timestamp stopStamp; // all information after that time will be ignored - useful to ensure delayed SBE results get accepeted or to include non-alarm data after alarm
  double navigationGPSSolutionRate; //[Hz] rate at which GPS solution should be provided 

  ////Some additional hooks we could use
  // Algorithm Parameters (left to LBL to auto configure for now)
  //double floatParameters[8];
  //int integerParamters[8];
  //bool booleanParameters[8];
};

#endif //#ifndef MSGDEFINITION_HPP

