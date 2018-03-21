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

  ZEDWrapper zed;
  
  int ac=0, debug;
  for(ac=1; ac<argc; ac++)
  {
    std::string tmp(argv[ac]);
    if(     tmp.compare("-d")==0) debug=std::stoi(std::string(argv[++ac]));
    else if(tmp.compare("--resolution")==0) zed.setImageResolution(std::stoi(std::string(argv[++ac])));
    else if(tmp.compare("--mapping")==0)    zed.setMappingFlag(true);
    else if(tmp.compare("--tracking")==0)   zed.setTrackingTopic(std::string(argv[++ac]));
    else if(tmp.compare("--left")==0)       zed.setLeftImageTopic(std::string(argv[++ac]));
    else if(tmp.compare("--right")==0)      zed.setRightImageTopic(std::string(argv[++ac]));
    else if(tmp.compare("--imu")==0)        zed.setImuTopic(std::string(argv[++ac]));
    else if(tmp.compare("--imu-rate")==0)   zed.setImuRate(std::stof(std::string(argv[++ac])));
    else if(tmp.compare("--mesh-rate")==0)  zed.setMeshRate(std::stof(std::string(argv[++ac])));
    else if(tmp.compare("--frame-rate")==0) zed.setFrameRate(std::stof(std::string(argv[++ac])));
    else if(tmp.compare("--length")==0)     zed.setBufferLength(std::stoi(std::string(argv[++ac])));
    else if(tmp.compare("--confidence")==0) zed.setConfidence(std::stoi(std::string(argv[++ac])));
    else if(tmp.compare("--mapping-resolution")==0) zed.setMappingResolution(std::stof(std::string(argv[++ac])));
    else if(tmp.compare("--mapping-range")==0)      zed.setMappingRange(std::stof(std::string(argv[++ac])));
    else if(tmp.compare("--quality")==0)            zed.setQuality(std::stoi(std::string(argv[++ac])));
    else break;
  }

  zed.startWrapperThread();

  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  std::cout << '\n' << "Server running enter reconfiguration command or 'q q' to terminate ..." << '\n' << ">> ";
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
