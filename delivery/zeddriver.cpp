/***********************************************************************************
 * Class to run a service that collects requested informations from the ZED-Mini
 * Camera in Shared Memory blocks. 
 * 
 * Inspired by https://github.com/stereolabs/zed-ros-wrapper
 *
 * Author: Marco Salathe <msalathe@lbl.gov>
 * Date:   March 2018
 *
 **********************************************************************************/

#include "ZEDWrapperPublic.hpp"
#include "ZEDMsgDefinition.hpp"

#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>
#include <string>
#include <vector>


void PrintThrift(const ZEDContextStreamDefinition &def)
{
  const double *pos, *rot;
  std::cout << "Component: " << std::endl;
  std::cout << "   Component Name: " << def.component.componentName << std::endl;
  std::cout << "   Vendor Name: " << def.component.vendorName << std::endl;
  std::cout << "   Serial Number: " << def.component.serialNumber << std::endl;
  std::cout << "Stream Format: " << def.streamFormat << std::endl;
  std::cout << "Stream Address: " << def.streamAddress << std::endl;
  std::cout << "Format Version: " << def.formatVersion << std::endl;
  std::cout << "Doucment URI: " << def.documentationURI << std::endl;
  std::cout << "ContextStreamConfiguration:" << std::endl;
  std::cout << "   Component Position and Orientation:" << std::endl;
  pos = def.configuration.componentPositionAndOrientation.position;
  std::cout << "      Position: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
  rot = def.configuration.componentPositionAndOrientation.rotation;
  std::cout << "      Rotation: " << rot[0] << " " << rot[1] << " " << rot[2] << " " << rot[3] << std::endl;
  std::cout << "   ContextVideoConfiguration:" << std::endl;
  std::cout << "      componentID: " << def.configuration.videoConfiguration.componentId << std::endl;
  std::cout << "      File Name: " << def.configuration.videoConfiguration.fileName << std::endl;
  std::cout << "      Frame per Second: " << def.configuration.videoConfiguration.framesPerSecond << std::endl;
  std::cout << "      vertical Resolution: " << def.configuration.videoConfiguration.verticalResolution << std::endl;
  std::cout << "      horizontal Resolution: " << def.configuration.videoConfiguration.horizontalResolution << std::endl;
  std::cout << "      Component Position and Orientation:" << std::endl;
  pos = def.configuration.videoConfiguration.componentPositionAndOrientation.position;
  std::cout << "         Position: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
  rot = def.configuration.videoConfiguration.componentPositionAndOrientation.rotation;
  std::cout << "         Rotation: " << rot[0] << " " << rot[1] << " " << rot[2] << " " << rot[3] << std::endl;
  std::cout << "      vertical FOV: " << def.configuration.videoConfiguration.verticalFOV << std::endl;
  std::cout << "      Horizontal FOV: " << def.configuration.videoConfiguration.horizontalFOV << std::endl;
  std::cout << "      Is Rectified: "  << def.configuration.videoConfiguration.isRectified << std::endl;
  std::cout << "      Is De Bayered: " << def.configuration.videoConfiguration.isDeBayered << std::endl;
  std::cout << "      CameraIntrinsics:" << std::endl;
  std::cout << "         cx cy: " << def.configuration.videoConfiguration.intrinsics.cx << " " << def.configuration.videoConfiguration.intrinsics.cy << std::endl;
  std::cout << "         fx fy: " << def.configuration.videoConfiguration.intrinsics.fx << " " << def.configuration.videoConfiguration.intrinsics.fy << std::endl;
  std::cout << "         K1 K2 ...: ";
  for (auto i = def.configuration.videoConfiguration.intrinsics.K.begin(); i != def.configuration.videoConfiguration.intrinsics.K.end(); ++i) std::cout << *i << " ";
  std::cout << std::endl;
  std::cout << "         P1 P2 ...: ";
  for (auto i = def.configuration.videoConfiguration.intrinsics.P.begin(); i != def.configuration.videoConfiguration.intrinsics.P.end(); ++i) std::cout << *i << " ";
  std::cout << std::endl;
  std::cout << "      Timestamp: " << def.configuration.videoConfiguration.timeStamp << std::endl;
  std::cout << "   Timestamp: " << def.configuration.timeStamp << std::endl;
};

int main(int argc, char** argv)
{

  //Create a ZEDWrapper object
  ZEDWrapperPublic zed;

  //Read the command line options (all rates are in Hz) provided as arguments to the script
  int ac=0, debug;
  for(ac=1; ac<argc; ac++)
  {
    std::string tmp(argv[ac]);
    if(     tmp.compare("-d")==0) debug=std::stoi(std::string(argv[++ac]));
    else if(tmp.compare("--mapping")==0)            zed.setMappingFlag(true);                                     //Enable mapping (the mesh can be later saved by calling saveMesh() )
    else if(tmp.compare("--tracking")==0)           zed.setTrackingTopic(std::string(argv[++ac]));                //Enable tracking and define the topic buffer name (arg2 == topicname)
    else if(tmp.compare("--left")==0)               zed.setLeftImageTopic(std::string(argv[++ac]));               //Define the left image topic buffer name (arg2 == topicname)
    else if(tmp.compare("--right")==0)              zed.setRightImageTopic(std::string(argv[++ac]));              //Define the right image topic buffer name (arg2 == topicname)
    else if(tmp.compare("--imu")==0)                zed.setImuTopic(std::string(argv[++ac]));                     //Set the imu topic buffer name  (arg2 == topicname)
    else if(tmp.compare("--depth")==0)              zed.setDepthTopic(std::string(argv[++ac]));                   //Set the depth image topic buffer name (arg2 == topicname, probably not useful)
    else if(tmp.compare("--imu-rate")==0)           zed.setImuRate(std::stof(std::string(argv[++ac])));           //IMU read out rate (arg2 == rate, default 200Hz)
    else if(tmp.compare("--frame-rate")==0)         zed.setFrameRate(std::stof(std::string(argv[++ac])));         //Image rate (arg2 == rate, default 30Hz)
    else if(tmp.compare("--camera-rate")==0)        zed.setCameraRate(std::stof(std::string(argv[++ac])));        //Camera rate (arg2 == rate, default 30Hz)
    else if(tmp.compare("--mesh-rate")==0)          zed.setMeshRate(std::stof(std::string(argv[++ac])));          //Mapping extraction rate (arg2 == rate, default 2Hz, only matter is mapping is enabled)
    else if(tmp.compare("--length")==0)             zed.setBufferLength(std::stoi(std::string(argv[++ac])));      //Buffer length with respect to the image buffer (arg2 == length, number of images in buffer, default 1)
    else if(tmp.compare("--resolution")==0)         zed.setImageResolution(std::stoi(std::string(argv[++ac])));   //Image resolution (arg2 == mode, 2 HD720, 3 VGA, default 3)
    else if(tmp.compare("--confidence")==0)         zed.setConfidence(std::stoi(std::string(argv[++ac])));        //Confidence threshold (arg2 == threshold, probably not useful)
    else if(tmp.compare("--mapping-resolution")==0) zed.setMappingResolution(std::stof(std::string(argv[++ac]))); //Mapping resolution in meters (arg2 == resolution, default 0.08)
    else if(tmp.compare("--mapping-range")==0)      zed.setMappingRange(std::stof(std::string(argv[++ac])));      //Mapping range in meters (arg2 == range, default 10, 20 might be better)
    else if(tmp.compare("--quality")==0)            zed.setQuality(std::stoi(std::string(argv[++ac])));           //Depth extraction method (arg2 == mode, 1 performance, 2 medium, 3 quality, 4 ultra, default 1)
    else if(tmp.compare("--norect")==0)             zed.doNotRectifyImages();                                     //do not rectify images (why would someone ever do that...)
    else if(tmp.compare("--local")==0)              zed.setLocalImageStorage(std::string(argv[++ac]), 
                                                      std::stoi(argv[++ac]), std::string(argv[++ac]));            //Store images to a location on the hard disk at a certain rate (arg2 == prefix, arg3 == rate, arg4 == endfix)
    else if(tmp.compare("--svo")==0)                zed.setSVOFile(std::string(argv[++ac]));                      //Replays a svo-file
    else break;
  }

  //start the ZedWrapper thread
  zed.startWrapperThread();

  //do a bit of sleeping to make sure the thread is completely started
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  std::cout << '\n' << "Server running enter reconfiguration command or 'q' to terminate ..." << '\n' << ">> ";

  //read in additional parameters from cin (these can be changed online)
  do
  {
    std::vector <std::string> a; std::string b, line;
    std::getline(std::cin, line);
    std::istringstream iss(line);
    while ( iss >> b) {
     a.push_back(b);
    }
    if     (a[0].compare("imu-rate")==0)   zed.setImuRate(std::stof(a[1]));                                      //Change the imu rate to a given value (arg2 == rate)
    else if(a[0].compare("mesh-rate")==0)  zed.setMeshRate(std::stof(a[1]));                                     //Change the mesh rate to a given value (arg2 == rate)
    else if(a[0].compare("frame-rate")==0) zed.setFrameRate(std::stof(a[1]));                                    //Change the frame rate to a given value (arg2 == rate)
    else if(a[0].compare("length")==0)     zed.setBufferLength(std::stoi(a[1]));                                 //Change the buffer lenght to a given value (arg2 == length)
    else if(a[0].compare("autocalib")==0)  zed.doAutoCalibration();                                              //Force an auto calibration (not sure if this does something)
    else if(a[0].compare("save-map")==0)                                                                         //Save the mesh to a file (arg2 == filename)
    {
      if(!zed.getMappingFlag()){ std::cout << "Mapping was not initialized, please restart server first" << std::endl; }
      zed.saveMesh(b);
    }
    else if(a[0].compare("info")==0)
    {
      std::vector <ZEDContextStreamDefinition> stream = zed.getZEDStreamDefinitions();
      for(auto i = stream.begin(); i != stream.end(); i++)
      {
        PrintThrift(*i);
        std::cout << std::endl;
      }
    }
    else if(a[0].compare("q")==0)          break;                                                                //Exit
    else std::cout << "Do not understand input, try again" << std::endl;
    std::cout << '\n' << ">> ";
  }
  while(true);

  return 0;
}
