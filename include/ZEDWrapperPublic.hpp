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

#ifndef ZEDWRAPPERPUBLIC_HPP
#define ZEDWRAPPERPUBLIC_HPP

//STL
#include <vector>
#include <string>

#include "ZEDMsgDefinition.hpp"

class ZEDWrapper;

//the actual zed wrapper class
class ZEDWrapperPublic {
  private:
    ZEDWrapper *internal;

  public:
    ZEDWrapperPublic();
    ~ZEDWrapperPublic();

    /*******************************************************************************
     * The main function, starts a seperate thread that controls the ZEDMini camera
     ******************************************************************************/
    
    //The thread is started on call of this function and will be terminated on
    //destruction of the class. If parameters need to be changed, it is necessary
    //right now to destroy the old object (let it go out of scope) and recreate
    //a new one with the new parameters and call this thread again.
    void startWrapperThread();

    /*******************************************************************************
     * Variables that need to be set before the call to the startWrapperThread()
     ******************************************************************************/

    //Define the left image topic buffer name and turn on buffering of these images
    void setLeftImageTopic(std::string name);

    //Define the right image topic buffer name and turn on buffering of these images
    void setRightImageTopic(std::string name);

    //Define the depth image buffer name and turn on buffering of these images
    //The mapping algorithm might need this
    void setDepthTopic(std::string name);

    //Enable ZEDMini tracking and define the name of the buffer where odometry data
    //will be streamed
    void setTrackingTopic(std::string name);

    //Set the imu topic buffer name and stores imu data to that buffer, the mapping 
    //algorithm might need this
    void setImuTopic(std::string name);

    //Turns on ZEDMini mapping, the mesh can be saved to a local file with the saveMesh 
    //call. We will most likely to external mapping, thus this won't be necessary at all
    //and should not be turned on.
    void setMappingFlag(bool on);

    //Forces the images streamed on LeftTopic and RightTopic not to be rectified. This
    //should never be done, as the ZED rectification is quite efficient and on the GPU.
    void doNotRectifyImages();

    //Define the image resolution: 4: Highest, 1: Lowest. Default is set to 1. Needs
    //probably not to be changed, maybe 2, never more.
    void setImageResolution(int reso);

    //Set confidence level of depth extraction. Default is 100, that is good for now.
    void setConfidence(int conf);

    //Set the mapping resolution, as we won't do ZEDMini mapping this is not used.
    //Value is in meter, default is 0.08, possible are also 0.01, 0.05
    void setMappingResolution(double mres);

    //Set the mapping range, as we won't do ZEDMini mapping this is not used.
    //Value is in meter, default is 10, 5 and 20 are possible values.
    void setMappingRange(double mran);

    //Set the Depth mode quality value. Default is 1==PERFORMANCE, possible values are
    //2==MEDIUM, 3==QUALITY, 4==ULTRA MODE, default is probably all we ever need.
    void setQuality(int qual);

    //Play back from a SVO file instead of live. As we do not use SVO files, this is
    //not useful, just for debugging.
    void setSVOFile(std::string svo_file);

    /*******************************************************************************
     * Functions that can be called while the thread is active
     ******************************************************************************/

    //Saves the mesh (mapping) that was observed/reconstructed at to this point to
    //a file on disk (ply-type)
    int saveMesh(std::string fname);

    //Define a file path to a location where images should be stored. The rate defines
    //the interval in Hz at which the images will be stored (should me small ~1Hz or so)
    //The endfix can be jpg, png. Needs to be only called once to enable the image storage.
    //can be turned of with setLocalImageStorageFlag(false) later on.
    void setLocalImageStorage(std::string prefix, double rate, std::string endfix = std::string("jpg"));

    //Set the rate at which images are stored to the buffer, default is 30, which should
    //be fine.
    void setFrameRate(double hertz);

    //Set a rate (Hz) at which the mapping mesh is extracted, won't be used.
    void setMeshRate(double hertz);

    //Set a rate (Hz) at which the Imu data is stored to the buffer. 200 is default
    //and good.
    void setImuRate(double hertz);

    //Set number of images that will be kept at the buffer, default is 1. So far
    //900 (at 30Hz this means 30sec) seems the way to go.
    void setBufferLength(int len);

    //Perfom a recalibration of the camera. Is done when started and should be done
    //automatically. Haven't seen any need for it yet.
    void doAutoCalibration();

    /*******************************************************************************
     * Getter methods, can be called any time
     ******************************************************************************/

    //See if Mapping is enabled (true) or not (false)
    bool getMappingFlag();

    //See if depth images are stored to buffers (true) or not (false)
    bool getDepthFlag();

    //See if tracking is on, that means odometry data stored to buffer (true)
    //or not (false)
    bool getTrackingFlag();

    //See if Imu data are stored to buffers (true) or not (false)
    bool getImuFlag();

    //See if left images are stored to a buffer (true) or not (false)
    bool getLeftImageFlag();

    //See if right images are stored to a buffer (true) or not (false)
    bool getRightImageFlag();

    //Extract a structure containing thrift information to be passed on
    //Returns a vector of all Context stream information, one element per
    //stream.
    std::vector <ZEDContextStreamDefinition> getZEDStreamDefinitions();
}; // class ZEDWrapperPublic


#endif //#ifndef ZEDWRAPPERPUBLIC_HPP
