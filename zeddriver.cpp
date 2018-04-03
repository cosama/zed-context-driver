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
#define USE_MANAGED_SHARED_MEMORY

#include <sl/Camera.hpp>
#include "ZEDWrapper.hpp"

#include <chrono>
#include <iostream>
#include <thread>
#include <string>
#include <vector>

//Simple function to display camera information
void PrintCameraInfo(CameraInfo info)
{
  int i=0;
  std::cout << "height: " << info.height << std::endl;
  std::cout << "width: " << info.width << std::endl;
  std::cout << "D:  [ "; for(i=0;i<5;i++)  std::cout << info.D[i] << " "; std::cout << "]" << std::endl;
  std::cout << "K:  [ "; for(i=0;i<9;i++)  std::cout << info.K[i] << " "; std::cout << "]" << std::endl;
  std::cout << "R:  [ "; for(i=0;i<9;i++)  std::cout << info.R[i] << " "; std::cout << "]" << std::endl;
  std::cout << "P:  [ "; for(i=0;i<12;i++) std::cout << info.P[i] << " "; std::cout << "]" << std::endl;
  std::cout << "Rectified: " << info.rectified << std::endl;
}

int main(int argc, char** argv)
{

  //Create a ZEDWrapper object
  ZEDWrapper zed;

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
    else if(a[0].compare("info-left")==0)  PrintCameraInfo(zed.getLeftCamaraInfo());                             //Display the left camera info
    else if(a[0].compare("info-right")==0) PrintCameraInfo(zed.getRightCamaraInfo());                            //Display the right camera info
    else if(a[0].compare("autocalib")==0)  zed.doAutoCalibration();                                              //Force an auto calibration (not sure if this does something)
    else if(a[0].compare("save-map")==0)                                                                         //Save the mesh to a file (arg2 == filename)
    {
      if(!zed.getMappingFlag()){ std::cout << "Mapping was not initialized, please restart server first" << std::endl; }
      zed.saveMesh(b);
    }
    else if(a[0].compare("q")==0)          break;                                                                //Exit
    else std::cout << "Do not understand input, try again" << std::endl;
    std::cout << '\n' << ">> ";
  }
  while(true);

  return 0;
}
