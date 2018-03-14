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

#include <iostream>
#include <ZEDWrapper.hpp>
#include <thread>
#include <string>

int main(int argc, char** argv)
{

  ZEDWrapper zed;
  
  int ac=0, debug;
  for(ac=1; ac<argc; ac++)
  {
    std::string tmp(argv[ac]);
    if(     tmp.compare("-d")==0) debug=std::stoi(std::string(argv[++ac]));
    else if(tmp.compare("--mapping")==0)    zed.MappingFlag(true);
    else if(tmp.compare("--tracking")==0)   zed.TrackingTopic(std::string(argv[++ac]));
    else if(tmp.compare("--left")==0)       zed.LeftImageTopic(std::string(argv[++ac]));
    else if(tmp.compare("--right")==0)      zed.RightImageTopic(std::string(argv[++ac]));
    else if(tmp.compare("--imu")==0)        zed.ImuTopic(std::string(argv[++ac]));
    else if(tmp.compare("--imu-rate")==0)   zed.ImuRate(std::stoi(std::string(argv[++ac])));
    else if(tmp.compare("--mesh-rate")==0)  zed.MeshRate(std::stoi(std::string(argv[++ac])));
    else if(tmp.compare("--frame-rate")==0) zed.FrameRate(std::stoi(std::string(argv[++ac])));
    else if(tmp.compare("--length")==0)     zed.BufferLength(std::stoi(std::string(argv[++ac])));
    else break;
  }

  zed.startWrapperThread();

  std::cout << '\n' << "Server running enter reconfiguration command or 'q' to terminate ...";
  std::string a, b;
  do
  {
    std::cin >> a >> b;
    if     (a.compare("imu-rate")==0)   zed.ImuRate(std::stoi(b));
    else if(a.compare("mesh-rate")==0)  zed.MeshRate(std::stoi(b));
    else if(a.compare("frame-rate")==0) zed.FrameRate(std::stoi(b));
    else if(a.compare("length")==0)     zed.BufferLength(std::stoi(b));
    else if(a.compare("q")==0)          break;
    else std::cout << "Do not understand input, try again" << std::endl;
  }
  while(true);
  //std::cin.ignore();

  return 0;
}
