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

//opencv is only used to define the matrix types, this dependency could be removed
#include <opencv2/core/core.hpp>

#include <sl/Camera.hpp>
#include <SharedBuffer.hpp>

#include <vector>
#include <iostream>
#include <chrono>
#include <thread>
typedef std::chrono::system_clock MyClock;

//largest common factor of 1280x720 and 672x376
#define IMAGE_SEND_BITS 768*4

//time stamp format similar to C's timeval
struct Timestamp{
  long long sec, usec;
};

//image header format
//the length of the header defines the block size
//of an image, the zed images need to be devided
//by this size
struct ImageHeaderMsg {
  int size;
  int cols;
  int rows;
  int type;
  Timestamp time;
  char pad[IMAGE_SEND_BITS-16];
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

int checkCameraReady(unsigned int serial_number) {
    int id = -1;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f)
        if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            id = it.id;
    return id;
};

sl::DeviceProperties getZEDFromSN(unsigned int serial_number) {
    sl::DeviceProperties prop;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f) {
        if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            prop = it;
    }
    return prop;
};


//Class to control that the loop is executed at a given frequency
class LoopTime {
  private:
    std::chrono::microseconds expected_cycle_time_;
    std::chrono::system_clock::time_point start_;

  public:
    LoopTime(int us_sleep): expected_cycle_time_(us_sleep)
    {
      start_ = std::chrono::system_clock::now();
    };

    void sleep()
    {
      MyClock::time_point expected_end = start_ + expected_cycle_time_;
      MyClock::time_point actual_end = MyClock::now();

      // detect backward jumps in time
      if (actual_end < start_)
      {
        expected_end = actual_end + expected_cycle_time_;
      }

      //std::cout << "expected minus actual " << std::chrono::duration_cast<std::chrono::microseconds>(expected_end - actual_end).count() << std::endl;

      //make sure to reset our start time
      start_ = expected_end;
      if(expected_end < actual_end)
        start_ = actual_end;

      std::this_thread::sleep_until(expected_end);
    };

    void set(int us_sleep)
    {
      expected_cycle_time_ = std::chrono::microseconds(us_sleep);
      start_ = std::chrono::system_clock::now();
    }
};

class ZEDWrapper {

  private:
    std::thread *device_poll_thread;

    SharedBuffer<ImageHeaderMsg> pub_left;
    SharedBuffer<ImageHeaderMsg> pub_right;
    SharedBuffer<PoseMsg> pub_odom;
    SharedBuffer<ImuMsg> pub_imu;

    // Launch file parameters
    int resolution;
    int quality;
    int sensing_mode;
    int gpu_id;
    int zed_id;
    bool depth_stabilization;
    double mapping_range;
    double mapping_resolution;
    int mapping_memory;

    // Tracking variables
    sl::Pose pose;
    sl::IMUData imud;

    // zed object
    sl::InitParameters param;
    sl::Camera zed;
    unsigned int serial_number;

    // flags
    int confidence;
    bool computeDepth;

    // parameters that can be changed
    // during execution
    bool limg_on;
    bool rimg_on;
    bool odom_on;
    bool imu_on;
    bool map_on;
    bool mesh_extracted;

    double frame_rate;
    double imu_rate;
    double mesh_rate;
    int max_buffer_length;

    bool run_wrapper;

    //we handle the mesh differently no shared memory here
    sl::Mesh mesh;


    /* \brief Publish the pose of the camera with a ros Publisher
     * \param base_transform : Transformation representing the camera pose from base frame
     * \param pub_odom : the publisher object to use
     * \param odom_frame_id : the id of the reference frame of the pose
     * \param t : the ros::Time to stamp the image
     */
    void publishOdom(sl::Pose odom_in, SharedBuffer<PoseMsg> &pub_odom, Timestamp t, int max_odom) {
      static PoseMsg odom_data;

      odom_data.time.sec  = t.sec;
      odom_data.time.usec = t.usec; // odom_frame

      // Add all value in odometry message
      sl::Translation pos = odom_in.getTranslation();
      odom_data.position[0]    = pos(2);
      odom_data.position[1]    = pos(0);
      odom_data.position[2]    = pos(1);
      sl::Orientation rot = odom_in.getOrientation();
      odom_data.orientation[0] =  rot(2);
      odom_data.orientation[1] = -rot(0);
      odom_data.orientation[2] = -rot(1);
      odom_data.orientation[3] =  rot(3);

      // Publish odometry message
      int buf_size = pub_odom.write(odom_data);
      if(buf_size>max_odom*sizeof(odom_data))
      {
        pub_odom.resize(max_odom*sizeof(odom_data));
      }
    }


    /* \brief Publish the IMU data of the mini camera with a ros Publisher
     * \param IMUDATA : IMUData from Mini
     * \param pub_imu_raw : the publisher object to use
     * \param imu_frame_id : the id of the reference frame of the imu
     * \param t : the ros::Time to stamp the image
     */
    void publishIMU(sl::IMUData imu_in, SharedBuffer<ImuMsg> &pub_imu, Timestamp t, int max_imu) {
      static ImuMsg imu_data;

      imu_data.time.sec  = t.sec;
      imu_data.time.usec = t.usec; // odom_frame

      sl::Orientation quat=imu_in.getOrientation();
      imu_data.orientation[0] =  quat(2);
      imu_data.orientation[1] = -quat(0);
      imu_data.orientation[2] = -quat(1);
      imu_data.orientation[3] =  quat(3);
      
      imu_data.angular_velocity[0] =  imu_in.angular_velocity[2]*0.01745329251;
      imu_data.angular_velocity[1] = -imu_in.angular_velocity[0]*0.01745329251;
      imu_data.angular_velocity[2] = -imu_in.angular_velocity[1]*0.01745329251;
      
      imu_data.linear_acceleration[0] =  imu_in.linear_acceleration[2];
      imu_data.linear_acceleration[1] = -imu_in.linear_acceleration[0];
      imu_data.linear_acceleration[2] = -imu_in.linear_acceleration[1];

      int buf_size = pub_imu.write(imu_data);
      if(buf_size>max_imu*sizeof(imu_data))
      {
        pub_imu.resize(max_imu*sizeof(imu_data));
      }
  }


    /* \brief Publish a cv::Mat image with a ros Publisher
     * \param img : the image to publish
     * \param pub_img : the publisher object to use
     * \param img_frame_id : the id of the reference frame of the image
     * \param t : the ros::Time to stamp the image
     */
    void publishImage(sl::Mat img_in, SharedBuffer<ImageHeaderMsg> &pub_img, Timestamp t, int max_img)
    {
      static int msgsize=sizeof(ImageHeaderMsg);
      if(img_in.getMemoryType() == sl::MEM_GPU) img_in.updateCPUfromGPU();

      ImageHeaderMsg hdr;
      hdr.size      = (int)(img_in.getHeight()*img_in.getWidth()*img_in.getPixelBytes());
      hdr.cols      = (int)(img_in.getWidth());
      hdr.rows      = (int)(img_in.getHeight());

      hdr.time.sec  = t.sec;
      hdr.time.usec = t.usec;

      switch (img_in.getDataType()) {
        case sl::MAT_TYPE_32F_C1:
          hdr.type = CV_32FC1;
          break;
        case sl::MAT_TYPE_32F_C2:
          hdr.type = CV_32FC2;
          break;
        case sl::MAT_TYPE_32F_C3:
          hdr.type = CV_32FC3;
          break;
        case sl::MAT_TYPE_32F_C4:
          hdr.type = CV_32FC4;
          break;
        case sl::MAT_TYPE_8U_C1:
          hdr.type = CV_8UC1;
          break;
        case sl::MAT_TYPE_8U_C2:
          hdr.type = CV_8UC2;
          break;
        case sl::MAT_TYPE_8U_C3:
          hdr.type = CV_8UC3;
          break;
        case sl::MAT_TYPE_8U_C4:
          hdr.type = CV_8UC4;
          break;
      }

      ImageHeaderMsg* img_ptr= (ImageHeaderMsg*)img_in.getPtr<sl::uchar1>(sl::MEM_CPU);
      int buf_size = pub_img.write({ &hdr, &hdr+1, img_ptr, img_ptr+hdr.size/msgsize });
      if(buf_size>max_img*(hdr.size/msgsize+1))
      {
        pub_img.resize(max_img*(hdr.size/msgsize+1));
      }

    }

    void device_poll() {

      MyClock::time_point old_t = MyClock::now();
      MyClock::time_point tmesh = MyClock::now();
      LoopTime loop_time(1e6/(frame_rate>imu_rate?frame_rate:imu_rate)); //micro seconds

      sl::ERROR_CODE grab_status;
      bool tracking_activated = false;
      bool mapping_activated = false;

      std::chrono::microseconds frame_time((int)(1e6 / frame_rate)); //in micro seconds
      MyClock::time_point frame_expected = old_t+frame_time;

      // Get the parameters of the ZED images
      int width = zed.getResolution().width;
      int height = zed.getResolution().height;

      sl::RuntimeParameters runParams;
      runParams.sensing_mode = static_cast<sl::SENSING_MODE> (sensing_mode);

      sl::TrackingParameters trackParams;

      sl::SpatialMappingParameters spatial_mapping_params;
      spatial_mapping_params.range_meter = mapping_range;           //sl::SpatialMappingParameters::get(static_cast<sl::SpatialMappingParameters::MAPPING_RANGE> (mapping_range));
      spatial_mapping_params.resolution_meter = mapping_resolution; //sl::SpatialMappingParameters::get(static_cast<sl::SpatialMappingParameters::MAPPING_RESOLUTION> (mapping_resolution));

      std::cout << "Mapping enabled with range " << spatial_mapping_params.range_meter << "m and resolution " << spatial_mapping_params.resolution_meter << "m" << std::endl;

      spatial_mapping_params.max_memory_usage = mapping_memory;
      spatial_mapping_params.save_texture = false;
      spatial_mapping_params.use_chunk_only = true;

      sl::Mat leftZEDMat, rightZEDMat;
      // Main loop
      while (run_wrapper) {
        // Check for subscribers
        bool runLoop = (limg_on + rimg_on + odom_on + imu_on + map_on);

        // Run the loop only if there is some subscribers
        if (runLoop) {

          auto time=MyClock::now();

          Timestamp tstamp;
          std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch());
          tstamp.sec  = sec.count();
          tstamp.usec = std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch() - sec).count();

          //update rates, so that they can be dynamically set
          loop_time.set(frame_rate>imu_rate?frame_rate:imu_rate);
          frame_time = std::chrono::microseconds((int)(1e6 / frame_rate));

          // Publish the IMU if someone has subscribed to
          if (imu_on) {
            zed.getIMUData(imud, sl::TIME_REFERENCE_CURRENT);
            publishIMU(imud, pub_imu, tstamp, max_buffer_length*imu_rate/frame_rate);
          }

          if(frame_expected>time){ 
            loop_time.sleep(); continue; //goon nothing to be done at this point
          }
          frame_expected = time + frame_time;

          if ((depth_stabilization || odom_on) && !tracking_activated) { //Start the tracking
            zed.enableTracking(trackParams);
            tracking_activated = true;
          } else if (!depth_stabilization && !odom_on && tracking_activated) { //Stop the tracking
            zed.disableTracking();
            tracking_activated = false;
          }
          computeDepth = map_on + odom_on; // Detect if one of the subscriber need to have the depth information

          if (computeDepth) {
            int actual_confidence = zed.getConfidenceThreshold();
            if (actual_confidence != confidence)
              zed.setConfidenceThreshold(confidence);
            runParams.enable_depth = true; // Ask to compute the depth
          } else
              runParams.enable_depth = false;

          if(map_on && !mapping_activated)
          {
            if(!tracking_activated) zed.enableTracking(trackParams);
              zed.enableSpatialMapping(spatial_mapping_params);
              sl::SPATIAL_MAPPING_STATE state=zed.getSpatialMappingState();
            if(state!=sl::SPATIAL_MAPPING_STATE::SPATIAL_MAPPING_STATE_NOT_ENABLED)
              mapping_activated = true;
          } else if (!map_on && mapping_activated) {
            zed.disableSpatialMapping();
            mapping_activated = false;
          }

          grab_status = zed.grab(runParams); // Ask to not compute the depth


          //std::cout << "Run wrapper " << zed.getCurrentFPS() << std::endl;

          //cout << toString(grab_status) << endl;
          if (grab_status != sl::ERROR_CODE::SUCCESS) { // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED
            if (grab_status == sl::ERROR_CODE_NOT_A_NEW_FRAME) {
              std::cout << "Wait for a new image to proceed" << std::endl;
            } else std::cout << toString(grab_status) << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            if ((time - old_t) > std::chrono::seconds(5)) {
              zed.close();

              std::cout << "Re-opening the ZED" << std::endl;
              sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
              while (err != sl::SUCCESS) {
                int id = checkCameraReady(serial_number);
                if (id > 0) {
                  param.camera_linux_id = id;
                  err = zed.open(param); // Try to initialize the ZED
                  std::cout << toString(err) << std::endl;
                } else std::cout << "Waiting for the ZED to be re-connected" << std::endl;
                  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
              }
              tracking_activated = false;
              if (odom_on) { //Start the tracking
                zed.enableTracking(trackParams);
                tracking_activated = true;
              }
            }
            continue;
          }
          old_t = MyClock::now();

          // Publish the image if someone has subscribed to
          if (limg_on) {
            zed.retrieveImage(leftZEDMat,  sl::VIEW_LEFT);
            publishImage(leftZEDMat,  pub_left,  tstamp, max_buffer_length);
          }
          if (rimg_on) {
            zed.retrieveImage(rightZEDMat, sl::VIEW_RIGHT);
            publishImage(rightZEDMat, pub_right, tstamp, max_buffer_length);
          }

          // Publish the odometry if someone has subscribed to
          if (odom_on) {
            sl::TRACKING_STATE tracking_error = zed.getPosition(pose);
            if(tracking_error!=sl::TRACKING_STATE_OK) std::cout << sl::trackingState2str(tracking_error) << std::endl;
            publishOdom(pose, pub_odom, tstamp, max_buffer_length);
          }

          // Ask for a mesh update if mesh_rate has expired since last request and grab mesh
          if(map_on && mapping_activated)
          {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(time - tmesh).count()>1e3/mesh_rate)
            {
                //std::cout << "Request mesh" << std::endl;
                zed.requestMeshAsync();
                tmesh = MyClock::now();
            }

            if (zed.getMeshRequestStatusAsync() == sl::SUCCESS) {
                zed.retrieveMeshAsync(mesh);
                mesh_extracted = true;
                //std::cout << "Mesh retrieved with " << mesh.getNumberOfTriangles() << " Triangles" << std::endl;
                //std::cout << "Mesh retrieved, current fps " << zed.getCurrentFPS() << std::endl;
            }
          }

          loop_time.sleep();

        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // No subscribers, we just wait
        }
      } // while loop
      zed.close();
    }

  public:
    int saveMesh(std::string fname){
      if(map_on)
      {
        mesh_extracted = false;
        while(mesh_extracted == false)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds((int)(1.0e3/mesh_rate)));
        }
        map_on = false;
        mesh.updateMeshFromChunkList();
        mesh.save(fname.c_str(), sl::MESH_FILE_PLY);
        std::cout << "Mesh saved as " << fname << std::endl;
        map_on = true;
        return 1;
      }
      std::cout << "Mesh extraction failed" << std::endl;
      return 0;
    }

    ZEDWrapper()
    {
      //default loop controls
      limg_on  = false;
      rimg_on  = false;
      odom_on  = false;
      imu_on   = false;
      map_on   = false;
      mesh_extracted = false;

      //default camera parameters
      resolution    = sl::RESOLUTION_VGA;
      quality       = sl::DEPTH_MODE_PERFORMANCE;
      sensing_mode  = sl::SENSING_MODE_STANDARD;
      mesh_rate     = 2;
      frame_rate    = 30;
      imu_rate      = 200;
      gpu_id        = -1;
      zed_id        = 0;
      serial_number = 0;
      confidence    = 100;

      //default mapping parameters
      mapping_range      = sl::SpatialMappingParameters::get(static_cast<sl::SpatialMappingParameters::MAPPING_RANGE> (sl::SpatialMappingParameters::MAPPING_RANGE_FAR));
      mapping_resolution = sl::SpatialMappingParameters::get(static_cast<sl::SpatialMappingParameters::MAPPING_RESOLUTION> (sl::SpatialMappingParameters::MAPPING_RESOLUTION_LOW));
      mapping_memory     = 2048; // it is in MBytes
      //default wrapper parameters
      run_wrapper = false;
      depth_stabilization = false;
      max_buffer_length = 1; //must be an even number for images


    }


    ~ZEDWrapper() {
      run_wrapper = false;
      device_poll_thread->join();
    }

    void startWrapperThread()
    {

      std::cout << "Initializing the wrapper server" << std::endl;
      if(limg_on) std::cout << "   Start Left Image Topic at " << pub_left.name() << std::endl;
      if(rimg_on) std::cout << "   Start Right Image Topic at " << pub_right.name() << std::endl;
      if(odom_on) std::cout << "   Start Odometry Topic at " << pub_odom.name() << std::endl;
      if(imu_on)  std::cout << "   Start Imu Topic at " << pub_left.name() << std::endl;
      std::cout << "   Mapping is turned " << (map_on?"on":"off") << std::endl;
      std::cout << "   Buffer length " << max_buffer_length << std::endl;
      std::cout << "   Frame rate " << frame_rate << std::endl;
      std::cout << "   Imu rate " << imu_rate << std::endl;
      std::cout << "   Mesh rate " << mesh_rate << std::endl;
      std::cout << "   Confidence " << confidence << std::endl;

      // Try to initialize the ZED
      param.camera_fps = frame_rate;
      param.camera_resolution = static_cast<sl::RESOLUTION> (resolution);
      if (serial_number == 0)
        param.camera_linux_id = zed_id;
      else {
        bool waiting_for_camera = true;
        while (waiting_for_camera) {
          sl::DeviceProperties prop = getZEDFromSN(serial_number);
          if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE) {
            std::string msg = "ZED SN" + std::to_string(serial_number) + " not detected ! Please connect this ZED";
            std::cout << msg.c_str() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
          } else {
            waiting_for_camera = false;
            param.camera_linux_id = prop.id;
          }
        }
      }

      param.coordinate_units  = sl::UNIT_METER;
      param.coordinate_system = sl::COORDINATE_SYSTEM_IMAGE;
      param.depth_mode = static_cast<sl::DEPTH_MODE> (quality);
      param.sdk_verbose = true;
      param.sdk_gpu_id = gpu_id;
      param.depth_stabilization = depth_stabilization;

      sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
      while (err != sl::SUCCESS) {
        err = zed.open(param);
        std::cout << toString(err) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }

      serial_number = zed.getCameraInformation().serial_number;

      run_wrapper = true;
      device_poll_thread = new std::thread(&ZEDWrapper::device_poll, this);
    }

    //all the setter and getter methods

    void setLeftImageTopic(std::string name)
    {
      pub_left.initialize(name);
      if(!pub_left.is_owner()){ pub_left.force_remove(); pub_left.initialize(); }
      limg_on = true;
    }

    void setRightImageTopic(std::string name)
    {
      pub_right.initialize(name);
      if(!pub_right.is_owner()){ pub_right.force_remove(); pub_right.initialize(); }
      rimg_on = true;
    }

    void setTrackingTopic(std::string name)
    {
      depth_stabilization = true;
      pub_odom.initialize(name);
      if(!pub_odom.is_owner()){ pub_odom.force_remove(); pub_odom.initialize(); }
      odom_on = true;
    }

    void setImuTopic(std::string name)
    {
      pub_imu.initialize(name);
      if(!pub_imu.is_owner()){ pub_imu.force_remove(); pub_imu.initialize(); }
      rimg_on = true;
    }

    inline void setMappingFlag(bool on)
    {
      depth_stabilization = true;
      map_on = on;
    }

    inline void setImageResolution(int reso)
    {
      resolution = reso;
    }

    inline void setConfidence(int conf)
    {
      confidence = conf;
    }

    inline void setMappingResolution(double mres)
    {
      mapping_resolution = mres;
    }

    inline void setMappingRange(double mran)
    {
      mapping_range = mran;
    }

    inline void setQuality(int qual)
    {
      quality = qual;
    }

    inline void setFrameRate(double hertz)
    {
      frame_rate = hertz;
      std::cout << "Frame rate set to " << frame_rate << std::endl;
    }

    inline void setMeshRate(double hertz)
    {
      mesh_rate = hertz;
      std::cout << "Mesh rate set to " << mesh_rate << std::endl;
    }

    inline void setImuRate(double hertz)
    {
      imu_rate = hertz;
      std::cout << "Imu rate set to " << imu_rate << std::endl;
    }

    inline void setBufferLength(int len)
    {
      max_buffer_length = len;
      std::cout << "Buffer length set to " << max_buffer_length << std::endl;
    }


    inline bool getMappingFlag()
    {
      return map_on;
    }


    inline bool getTrackingFlag()
    {
      return odom_on;
    }

    inline bool getImuFlag()
    {
      return imu_on;
    }

    inline bool getLeftImageFlag()
    {
      return limg_on;
    }

    inline bool getRightImageFlag()
    {
      return rimg_on;
    }
}; // class ZEDWrapper

