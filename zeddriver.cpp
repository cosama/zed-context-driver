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
#include <ZEDWrapper.hpp>

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

int main(int argc, char** argv)
{

  //Create a ZEDWrapper object
  ZEDWrapper zed;

  //Read the command line options
  int ac=0, debug;
  for(ac=1; ac<argc; ac++)
  {
    std::string tmp(argv[ac]);
    if(     tmp.compare("-d")==0) debug=std::stoi(std::string(argv[++ac]));
    else if(tmp.compare("--mapping")==0)            zed.setMappingFlag(true);                                     //Enable mapping (the mesh can be later saved by calling saveMesh() )
    else if(tmp.compare("--tracking")==0)           zed.setTrackingTopic(std::string(argv[++ac]));                //Enable tracking and define the topic buffer name
    else if(tmp.compare("--left")==0)               zed.setLeftImageTopic(std::string(argv[++ac]));               //Define the left image topic buffer name
    else if(tmp.compare("--right")==0)              zed.setRightImageTopic(std::string(argv[++ac]));              //Define the right image topic buffer name
    else if(tmp.compare("--imu")==0)                zed.setImuTopic(std::string(argv[++ac]));                     //Set the imu topic buffer name
    else if(tmp.compare("--depth")==0)              zed.setDepthTopic(std::string(argv[++ac]));                   //Set the depth image topic buffer name (probably not useful)
    else if(tmp.compare("--imu-rate")==0)           zed.setImuRate(std::stof(std::string(argv[++ac])));           //IMU read out rate (default 200Hz)
    else if(tmp.compare("--frame-rate")==0)         zed.setFrameRate(std::stof(std::string(argv[++ac])));         //Image rate (default 30Hz)
    else if(tmp.compare("--mesh-rate")==0)          zed.setMeshRate(std::stof(std::string(argv[++ac])));          //Mapping extraction rate (default 2Hz, only matter is mapping is enabled)
    else if(tmp.compare("--length")==0)             zed.setBufferLength(std::stoi(std::string(argv[++ac])));      //Buffer length with respect to the image buffer (number of images in buffer, default 1)
    else if(tmp.compare("--resolution")==0)         zed.setImageResolution(std::stoi(std::string(argv[++ac])));   //Image resolution (2 HD720, 3 VGA, default 3)
    else if(tmp.compare("--confidence")==0)         zed.setConfidence(std::stoi(std::string(argv[++ac])));        //Confidence threshold (probably not useful)
    else if(tmp.compare("--mapping-resolution")==0) zed.setMappingResolution(std::stof(std::string(argv[++ac]))); //Mapping resolution in meters (default 0.08)
    else if(tmp.compare("--mapping-range")==0)      zed.setMappingRange(std::stof(std::string(argv[++ac])));      //Mapping range in meters (default 10, 20 might be better)
    else if(tmp.compare("--quality")==0)            zed.setQuality(std::stoi(std::string(argv[++ac])));           //Depth extraction method (1 performance, 2 medium, 3 quality, 4 ultra, default 1)
    else if(tmp.compare("--norect")==0)             zed.doNotRectifyImages();                                     //do not rectify images (why would someone ever do that...)
    else break;
  }

  //start the ZedWrapper thread
  zed.startWrapperThread();

  //do a bit of sleeping to make sure the thread is completely started
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  std::cout << '\n' << "Server running enter reconfiguration command or 'q q' to terminate ..." << '\n' << ">> ";

  //read in additional parameters from cin (these can be changed online)
  std::string a, b;
  do
  {
    std::cin >> a >> b;
    if     (a.compare("imu-rate")==0)   zed.setImuRate(std::stof(b));
    else if(a.compare("mesh-rate")==0)  zed.setMeshRate(std::stof(b));
    else if(a.compare("frame-rate")==0) zed.setFrameRate(std::stof(b));
    else if(a.compare("length")==0)     zed.setBufferLength(std::stoi(b));
    else if(a.compare("save-map")==0)
    {
      if(!zed.getMappingFlag()){ std::cout << "Mapping was not initialized, please restart server first" << std::endl; }
      zed.saveMesh(b);
    }
    else if(a.compare("q")==0)          break;
    else std::cout << "Do not understand input, try again" << std::endl;
    std::cout << '\n' << ">> ";
  }
  while(true);

  return 0;
}
