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

#include "ZEDWrapperPublic.hpp"
#include "ZEDWrapper.hpp"


int ZEDWrapperPublic::saveMesh(std::string fname){
    return internal->saveMesh(fname);
};

ZEDWrapperPublic::ZEDWrapperPublic()
{
  internal = new ZEDWrapper();
};

ZEDWrapperPublic::~ZEDWrapperPublic() {
  delete internal;
};

void ZEDWrapperPublic::startWrapperThread()
{
  internal->startWrapperThread();
};

void ZEDWrapperPublic::setLeftImageTopic(std::string name)
{
  internal->setLeftImageTopic(name);
};

void ZEDWrapperPublic::setRightImageTopic(std::string name)
{
  internal->setRightImageTopic(name);
};

void ZEDWrapperPublic::setDepthTopic(std::string name)
{
  internal->setDepthTopic(name);
};

void ZEDWrapperPublic::setTrackingTopic(std::string name)
{
  internal->setTrackingTopic(name);
};

void ZEDWrapperPublic::setImuTopic(std::string name)
{
  internal->setImuTopic(name);
};

void ZEDWrapperPublic::setLocalImageStorage(std::string prefix, double rate, std::string endfix)
{
  internal->setLocalImageStorage(prefix, rate, endfix);
};

void ZEDWrapperPublic::setMappingFlag(bool on)
{
  internal->setMappingFlag(on);
};

void ZEDWrapperPublic::doNotRectifyImages()
{
  internal->doNotRectifyImages();
};

void ZEDWrapperPublic::doAutoCalibration()
{
  internal->doAutoCalibration();
};

void ZEDWrapperPublic::setImageResolution(int reso)
{
  internal->setImageResolution(reso);
};

void ZEDWrapperPublic::setConfidence(int conf)
{
  internal->setConfidence(conf);
};

void ZEDWrapperPublic::setMappingResolution(double mres)
{
  internal->setMappingResolution(mres);
};

void ZEDWrapperPublic::setMappingRange(double mran)
{
  internal->setMappingRange(mran);
};

void ZEDWrapperPublic::setQuality(int qual)
{
  internal->setQuality(qual);
};

void ZEDWrapperPublic::setFrameRate(double hertz)
{
  internal->setFrameRate(hertz);
};

void ZEDWrapperPublic::setMeshRate(double hertz)
{
  internal->setMeshRate(hertz);
};

void ZEDWrapperPublic::setImuRate(double hertz)
{
  internal->setImuRate(hertz);
};

void ZEDWrapperPublic::setBufferLength(int len)
{
  internal->setBufferLength(len);
};

void ZEDWrapperPublic::setSVOFile(std::string svo_file)
{
  internal->setSVOFile(svo_file);
};

bool ZEDWrapperPublic::getMappingFlag()
{
  return internal->getMappingFlag();
};

bool ZEDWrapperPublic::getDepthFlag()
{
  return internal->getDepthFlag();
};

bool ZEDWrapperPublic::getTrackingFlag()
{
  return internal->getImuFlag();
};

bool ZEDWrapperPublic::getImuFlag()
{
  return internal->getImuFlag();
};

bool ZEDWrapperPublic::getLeftImageFlag()
{
  return internal->getLeftImageFlag();
};

bool ZEDWrapperPublic::getRightImageFlag()
{
  return internal->getRightImageFlag();
};

std::vector <ZEDContextStreamDefinition> ZEDWrapperPublic::getZEDStreamDefinitions()
{
  return internal->getZEDStreamDefinitions();
};
