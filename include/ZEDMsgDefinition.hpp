/***********************************************************************************
 * Header file that defines the thrift message structures for the ZEDMini
 * 
 * Author: Marco Salathe <msalathe@lbl.gov>
 * Date:   March 2018
 *
 **********************************************************************************/

#ifndef ZEDMSGDEFINITION_HPP
#define ZEDMSGDEFINITION_HPP

//All the below definition miss the UUID component of the respective 
//messages. See WIND-Thrift docu for where these are necessary. However, 
//ZEDContextVideoConfiguration has a compnentID, which is equals to <=2
//for a ContextVideoConfiguration and >=3 for a Context3DConfiguration.
//The coordinate frame is the camera image frame of the left camera, see
//https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
//for more details.

//Thrift-like GridPositionAndOrientation
struct ZEDGridPositionAndOrientation {
  double position[3];
  double rotation[4];
};

//Thrift-like CameraIntrinsics
struct ZEDCameraIntrinsics {
  double cx, cy; //horizontal, vertical optical center
  double fx, fy; //x/y vocal length
  double K[3];   //radial distortion coefficients
  double P[2];   //tangential distortion coefficients
};

//Thrift-like ComponentDefinition
struct ZEDComponentDefinition {
	std::string componentName;
	std::string vendorName;
	std::string serialNumber;
};

//Thrift-like ContextVideoConfiguration
struct ZEDContextVideoConfiguration {
  int componentId;
  std::string fileName;
  double framesPerSecond;
  double verticalResolution;
  double horizontalResolution;
  ZEDGridPositionAndOrientation componentPositionAndOrientation;
  double verticalFOV;
  double horizontalFOV;
  bool isRectified;
  bool isDeBayered;
  ZEDCameraIntrinsics intrinsics;
  long long timeStamp;
};

//Thrift-like ContextStreamConfiguration
struct ZEDContextStreamConfiguration
{
  ZEDGridPositionAndOrientation componentPositionAndOrientation;
  ZEDContextVideoConfiguration videoConfiguration;
  long long timeStamp;
};

//Thrift-like ContextStreamDefinition
struct ZEDContextStreamDefinition {
  ZEDComponentDefinition component;
  std::string streamFormat;
  std::string streamAddress;
  std::string formatVersion;
  std::string documentationURI;
  ZEDContextStreamConfiguration configuration;
};

#endif //#ifndef ZEDMSGDEFINITION_HPP
