/***********************************************************************************
 * Class to run a service that collects requested informations from the ZED-Mini
 * Camera in Shared Memory blocks. 
 * 
 * Inspired by https://github.com/stereolabs/zed-ros-wrapper
 *
 * Author:  Marco Salathe <msalathe@lbl.gov>
 * Date:    March 2018
 * License: See License.txt in parent folder of this repository.
 *
 **********************************************************************************/

#ifndef ZEDWRAPPER_HPP
#define ZEDWRAPPER_HPP

//Shared memory buffer wrapper
#include "SimpleBuffer.hpp"

//Message definitions
#include "MsgDefinition.hpp"
#include "ZEDMsgDefinition.hpp"

//OpenCV is to define the matrix types and to save images to disk
#include <opencv2/opencv.hpp>

//ZED SDK
#include <sl/Camera.hpp>

//STL
#include <vector>
#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>

typedef std::chrono::system_clock MyClock;

//Some general purpose functions from the zed-ros-wrapper
int checkCameraReady(unsigned int serial_number)
{
    int id = -1;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f)
        if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            id = it.id;
    return id;
};

sl::DeviceProperties getZEDFromSN(unsigned int serial_number)
{
    sl::DeviceProperties prop;
    auto f = sl::Camera::getDeviceList();
    for (auto &it : f)
    {
        if (it.serial_number == serial_number && it.camera_state == sl::CAMERA_STATE::CAMERA_STATE_AVAILABLE)
            prop = it;
    }
    return prop;
};

int saveZEDMattoImage(std::string prefix, sl::Mat mat, Timestamp time, std::string endfix=std::string("jpg"))
{
  if (mat.getMemoryType() == sl::MEM_GPU)
      mat.updateCPUfromGPU();

  int cvType;
  switch (mat.getDataType())
  {
    case sl::MAT_TYPE_32F_C1:
      cvType = CV_32FC1;
      break;
    case sl::MAT_TYPE_32F_C2:
      cvType = CV_32FC2;
      break;
    case sl::MAT_TYPE_32F_C3:
      cvType = CV_32FC3;
      break;
    case sl::MAT_TYPE_32F_C4:
      cvType = CV_32FC4;
      break;
    case sl::MAT_TYPE_8U_C1:
      cvType = CV_8UC1;
      break;
    case sl::MAT_TYPE_8U_C2:
      cvType = CV_8UC2;
      break;
    case sl::MAT_TYPE_8U_C3:
      cvType = CV_8UC3;
      break;
    case sl::MAT_TYPE_8U_C4:
      cvType = CV_8UC4;
      break;
  }
  cv::Mat cvmat((int) mat.getHeight(), (int) mat.getWidth(), cvType, mat.getPtr<sl::uchar1>(sl::MEM_CPU), mat.getStepBytes(sl::MEM_CPU));
  std::ostringstream stringStream;
  stringStream << prefix << time.sec << "." << std::setfill('0') << std::setw(6) << time.usec << "." << endfix; //I can believe it is 2018 and I have to do this...
  cv::imwrite(stringStream.str(), cvmat);
  return 1;
}

cv::Mat convertRodrigues(sl::float3 r)
{
  double theta = sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
  cv::Mat R = cv::Mat::eye(3, 3, CV_32F);

  if (theta < DBL_EPSILON)
  {
    return R;
  }
  else
  {
    double c = cos(theta);
    double s = sin(theta);
    double c1 = 1. - c;
    double itheta = theta ? 1. / theta : 0.;

    r *= itheta;

    cv::Mat rrt = cv::Mat::eye(3, 3, CV_32F);
    float* p = (float*) rrt.data;
    p[0] = r.x * r.x;
    p[1] = r.x * r.y;
    p[2] = r.x * r.z;
    p[3] = r.x * r.y;
    p[4] = r.y * r.y;
    p[5] = r.y * r.z;
    p[6] = r.x * r.z;
    p[7] = r.y * r.z;
    p[8] = r.z * r.z;

    cv::Mat r_x = cv::Mat::eye(3, 3, CV_32F);
    p = (float*) r_x.data;
    p[0] = 0;
    p[1] = -r.z;
    p[2] = r.y;
    p[3] = r.z;
    p[4] = 0;
    p[5] = -r.x;
    p[6] = -r.y;
    p[7] = r.x;
    p[8] = 0;

    // R = cos(theta)*I + (1 - cos(theta))*r*rT + sin(theta)*[r_x]
    R = c * cv::Mat::eye(3, 3, CV_32F) + c1 * rrt + s*r_x;
  }
  return R;
}

CameraInfo fillCamInfo(sl::Camera& zed, sl::VIEW view)
{

  CameraInfo cam_info = {0}; //set everything to zero

  cam_info.width  = zed.getResolution().width;
  cam_info.height = zed.getResolution().height;

  sl::CalibrationParameters zedParam;
  sl::CameraParameters camParam;
  cv::Mat R_; float *p;
  for (int i = 0; i < 3; i++) cam_info.R[i + i * 3] = 1;

  switch(view)
  {
    case sl::VIEW_LEFT:
      zedParam = zed.getCameraInformation().calibration_parameters;
      camParam = zedParam.left_cam;
      cam_info.rectified = true;
      break;
    case sl::VIEW_RIGHT:
      zedParam = zed.getCameraInformation().calibration_parameters;
      camParam = zedParam.right_cam;
      cam_info.P[3] = (-1 * zedParam.left_cam.fx * zedParam.T[0]);
      cam_info.rectified = true;
      break;
    case sl::VIEW_LEFT_UNRECTIFIED:
      zedParam = zed.getCameraInformation().calibration_parameters_raw;
      camParam = zedParam.left_cam;
      cam_info.rectified = false;
      break;
    case sl::VIEW_RIGHT_UNRECTIFIED:
      zedParam = zed.getCameraInformation().calibration_parameters_raw;
      camParam = zedParam.right_cam;
      cam_info.P[3] = (-1 * zedParam.left_cam.fx * zedParam.T[0]);
      R_ = convertRodrigues(zedParam.R); p = (float*)R_.data;
      for (int i = 0; i < 9; i++) cam_info.R[i] = p[i];
      cam_info.rectified = false;
    default:
      break;
  }

  cam_info.D[0] = camParam.disto[0];   // k1
  cam_info.D[1] = camParam.disto[1];   // k2
  cam_info.D[2] = camParam.disto[4];   // k3
  cam_info.D[3] = camParam.disto[2];   // p1
  cam_info.D[4] = camParam.disto[3];   // p2

  cam_info.K[0] = cam_info.P[0]  = camParam.fx;
  cam_info.K[2] = cam_info.P[2]  = camParam.cx;
  cam_info.K[4] = cam_info.P[5]  = camParam.fy;
  cam_info.K[5] = cam_info.P[6]  = camParam.cy;
  cam_info.K[8] = cam_info.P[10] = 1.0;

  return cam_info;
}

//Class to control that the loop is executed at a given frequency
class LoopTime{
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

//the actual zed wrapper class
class ZEDWrapper {

  private:
    std::thread *device_poll_thread;

    SimpleBuffer<ImageHeaderMsg> pub_left;
    SimpleBuffer<ImageHeaderMsg> pub_right;
    SimpleBuffer<ImageHeaderMsg> pub_depth;
    SimpleBuffer<PoseMsg> pub_odom;
    SimpleBuffer<ImuMsg> pub_imu;

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
    bool depth_on;
    bool reset;

    sl::VIEW lv, rv;
    CameraInfo info_left, info_right;
    sl::Transform imu_trans;
    bool autocalibration;

    double frame_rate;
    double imu_rate;
    double mesh_rate;
    double camera_rate;
    int max_buffer_length;

    bool bimg_on;
    std::string bimg_prefix;
    std::string bimg_endfix;
    double bimg_rate;


    bool run_wrapper;

    //we handle the mesh differently no shared memory here
    sl::Mesh mesh;

    //svo playback for debugging
    std::string svo_filename;

    //publish odometry data to a shared memory buffer
    void publishOdom(sl::Pose odom_in, SimpleBuffer<PoseMsg> &pub_odom, Timestamp t, int max_odom)
    {
      static PoseMsg odom_data;

      odom_data.time.sec  = t.sec;
      odom_data.time.usec = t.usec; // odom_frame

      // Add all value in odometry message
      sl::Translation pos = odom_in.getTranslation();
      odom_data.position[0]    = pos(0); //2
      odom_data.position[1]    = pos(1); //0
      odom_data.position[2]    = pos(2); //1
      sl::Orientation rot = odom_in.getOrientation();
      odom_data.orientation[0] =  rot(0); //2
      odom_data.orientation[1] =  rot(1); //-0
      odom_data.orientation[2] =  rot(2); //-1
      odom_data.orientation[3] =  rot(3); //3
      odom_data.confidence = odom_in.pose_confidence;

      // Publish odometry message
      pub_odom.write_simple(odom_data, max_odom);
    }


    // publish imu data to a shared memory buffer
    void publishIMU(sl::IMUData imu_in, SimpleBuffer<ImuMsg> &pub_imu, Timestamp t, int max_imu)
    {
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

      // Publish imu message
      pub_imu.write_simple(imu_data, max_imu);
    }


    // publish an image (stored as an ZED SDK Matrix) to a shared memory buffer
    void publishImage(sl::Mat img_in, SimpleBuffer<ImageHeaderMsg> &pub_img, Timestamp t, int max_img)
    {
      static int msgsize=sizeof(ImageHeaderMsg);
      static int cur_img=0;

      if(img_in.getMemoryType() == sl::MEM_GPU) img_in.updateCPUfromGPU();

      ImageHeaderMsg hdr;
      hdr.size       = (int)(img_in.getHeight()*img_in.getWidth()*img_in.getPixelBytes());
      hdr.block_size = msgsize;
      hdr.block_nmb  = hdr.size/hdr.block_size;
      hdr.cols       = (int)(img_in.getWidth());
      hdr.rows       = (int)(img_in.getHeight());

      hdr.time.sec  = t.sec;
      hdr.time.usec = t.usec;

      switch (img_in.getDataType())
      {
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

      hdr.data = (void*)img_in.getPtr<sl::uchar1>(sl::MEM_CPU);
      pub_img.write_simple(hdr, max_img);
    }

    void device_poll()
    {
      if(limg_on) pub_left.resize(0);
      if(rimg_on) pub_right.resize(0);
      if(depth_on) pub_depth.resize(0);
      if(odom_on) pub_odom.resize(0);
      if(imu_on) pub_imu.resize(0);

      MyClock::time_point old_t = MyClock::now();
      MyClock::time_point tmesh = MyClock::now();
      MyClock::time_point tbimg = MyClock::now();
      LoopTime loop_time(1e6/(camera_rate>imu_rate?camera_rate:imu_rate)); //micro seconds

      MyClock::time_point svo_time=MyClock::now();
  
      sl::ERROR_CODE grab_status;
      bool tracking_activated = false;
      bool mapping_activated = false;
      bool from_svo = (zed.getSVONumberOfFrames()>0);
      int lost_frames = 0;

      std::chrono::microseconds camera_time((int)(1e6 / camera_rate)); //in micro seconds
      MyClock::time_point camera_expected = old_t+camera_time;

      std::chrono::microseconds frame_time((int)(1e6 / frame_rate)); //in micro seconds
      MyClock::time_point frame_expected = old_t+frame_time;

      // Get the parameters of the ZED images
      //int width = zed.getResolution().width;
      //int height = zed.getResolution().height;

      sl::RuntimeParameters runParams;
      runParams.sensing_mode = static_cast<sl::SENSING_MODE> (sensing_mode);

      sl::TrackingParameters trackParams;

      sl::SpatialMappingParameters spatial_mapping_params;
      spatial_mapping_params.range_meter = mapping_range;           //sl::SpatialMappingParameters::get(static_cast<sl::SpatialMappingParameters::MAPPING_RANGE> (mapping_range));
      spatial_mapping_params.resolution_meter = mapping_resolution; //sl::SpatialMappingParameters::get(static_cast<sl::SpatialMappingParameters::MAPPING_RESOLUTION> (mapping_resolution));

      if(map_on) std::cout << "Mapping enabled with range " << spatial_mapping_params.range_meter << "m and resolution " << spatial_mapping_params.resolution_meter << "m" << std::endl;

      spatial_mapping_params.max_memory_usage = mapping_memory;
      spatial_mapping_params.save_texture = false;
      spatial_mapping_params.use_chunk_only = true;
      
      sl::Mat leftZEDMat, rightZEDMat, depthZEDMat;
      // Main loop
      while (run_wrapper)
      {
        // Check for subscribers
        bool runLoop = (limg_on + rimg_on + odom_on + imu_on + map_on + bimg_on);

        // Run the loop only if there is some subscribers
        if (runLoop)
        {
          auto time=MyClock::now();
          if(from_svo)
          {
            sl::timeStamp ts_old=zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);
            grab_status = zed.grab(runParams); // Ask to not compute the depth
            sl::timeStamp ts_new=zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);
            svo_time = svo_time + std::chrono::nanoseconds(ts_new - ts_old); //define when we have to get the next frame
            time = svo_time;
            loop_time.set(0); //read it in continously
          }

          Timestamp tstamp;
          std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch());
          tstamp.sec  = sec.count();
          tstamp.usec = std::chrono::duration_cast<std::chrono::microseconds>(time.time_since_epoch() - sec).count();

          if(from_svo)
          {
             sl::timeStamp ts_new=zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);
             tstamp.sec = (int)(ts_new/1e9);
             tstamp.usec = (int)(ts_new/1e3 - tstamp.sec*1e6);
          }
          //update rates, so that they can be dynamically set
          loop_time.set(1e6/(camera_rate>imu_rate?camera_rate:imu_rate));

          // Publish the IMU if someone has subscribed to
          if (imu_on)
          {
            zed.getIMUData(imud, sl::TIME_REFERENCE_CURRENT);
            publishIMU(imud, pub_imu, tstamp, max_buffer_length*imu_rate/frame_rate);
          }

          //synchronize time stamps of svo with current time
          //if(zed.getSVONumberOfFrames()>0)
          //{
            //if(time>svo_time)
            //{
            //  sl::timeStamp ts_old=zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);
            //  grab_status = zed.grab(runParams); // Ask to not compute the depth
            //  sl::timeStamp ts_new=zed.getTimestamp(sl::TIME_REFERENCE::TIME_REFERENCE_IMAGE);
              //long long run_time = std::chrono::duration_cast<std::chrono::nanoseconds>(time - svo_time).count();
              //long long dlt_time = ts_new-ts_old;
              //if(dlt_time<run_time)
              //{
              //  int old_pos = zed.getSVOPosition();
              //  int skips = run_time/dlt_time+1;
              //  std::cout << old_pos << " " << skips << " " << run_time << " " << dlt_time << std::endl;
              //  if(old_pos+skips>zed.getSVONumberOfFrames()) skips=zed.getSVONumberOfFrames()-old_pos;
              //  zed.setSVOPosition(old_pos+skips);
              //  ts_new+=skips*dlt_time;
              //}
              //time = time + std::chrono::nanoseconds(ts_new - ts_old); //define when we have to get the next frame
            //}
          //}

          if(camera_expected>time)
          {
            loop_time.sleep(); continue; //goon nothing to be done at this point
          }
          //camera_time = std::chrono::microseconds((int)(1e6 / camera_rate)); //camera rate can not be changed
          camera_expected = time + camera_time;

          if ((depth_stabilization || odom_on) && !tracking_activated)
          { //Start the tracking
            zed.enableTracking(trackParams);
            tracking_activated = true;
          }
          else if (!depth_stabilization && !odom_on && tracking_activated)
          { //Stop the tracking
            zed.disableTracking();
            tracking_activated = false;
          }
          computeDepth = map_on + odom_on + depth_on; // Detect if one of the subscriber need to have the depth information

          if (computeDepth)
            runParams.enable_depth = true; // Ask to compute the depth
          else
            runParams.enable_depth = false;

          if(map_on && !mapping_activated)
          {
            if(!tracking_activated) zed.enableTracking(trackParams);
              zed.enableSpatialMapping(spatial_mapping_params);
              sl::SPATIAL_MAPPING_STATE state=zed.getSpatialMappingState();
            if(state!=sl::SPATIAL_MAPPING_STATE::SPATIAL_MAPPING_STATE_NOT_ENABLED)
              mapping_activated = true;
          }
          else if (!map_on && mapping_activated)
          {
            zed.disableSpatialMapping();
            mapping_activated = false;
          }

          grab_status = zed.grab(runParams); // Ask to not compute the depth

          //std::cout << "Run wrapper " << zed.getCurrentFPS() << std::endl;
          //cout << toString(grab_status) << endl;
          if (grab_status != sl::ERROR_CODE::SUCCESS)
          { // Detect if a error occurred (for example: the zed have been disconnected) and re-initialize the ZED
            if (grab_status == sl::ERROR_CODE_NOT_A_NEW_FRAME)
            {
              if(from_svo && zed.getSVONumberOfFrames() == zed.getSVOPosition())
              {
                std::cout << "End of file reached, stop thread (do not terminate topics)" << std::endl;
                run_wrapper = false;
                break;
              }
              std::cout << "Wait for a new image to proceed" << std::endl;
            } 
            else std::cout << toString(grab_status) << std::endl;

            std::this_thread::sleep_for(std::chrono::milliseconds(2));

            if ((time - old_t) > std::chrono::seconds(5))
            {
              zed.close();

              std::cout << "Re-opening the ZED" << std::endl;
              sl::ERROR_CODE err = sl::ERROR_CODE_CAMERA_NOT_DETECTED;
              while (err != sl::SUCCESS)
              {
                int id = checkCameraReady(serial_number); 
                if (id > 0)
                {
                  param.camera_linux_id = id;
                  err = zed.open(param); // Try to initialize the ZED
                  std::cout << toString(err) << std::endl;
                } else std::cout << "Waiting for the ZED to be re-connected" << std::endl;
                  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
              }
              tracking_activated = false;
              if (odom_on)
              { //Start the tracking
                zed.enableTracking(trackParams);
                tracking_activated = true;
              }
            }
            continue;
          }
          old_t = time; //MyClock::now();

          if(frame_expected>time)
          {
            continue; //goon nothing to be done at this point
          }
          frame_time = std::chrono::microseconds((int)(1e6 / frame_rate)); //camera rate can not be changed
          frame_expected = time + frame_time;


          // Publish the image if someone has subscribed to
          if (limg_on)
          {
            zed.retrieveImage(leftZEDMat, lv);
            publishImage(leftZEDMat,  pub_left,  tstamp, max_buffer_length);
          }
          if (rimg_on)
          {
            zed.retrieveImage(rightZEDMat, rv);
            publishImage(rightZEDMat, pub_right, tstamp, max_buffer_length);
          }
          if (depth_on)
          {
            zed.retrieveMeasure(depthZEDMat, sl::MEASURE_DEPTH);
            publishImage(depthZEDMat, pub_depth, tstamp, max_buffer_length);
          }

          // Publish the odometry if someone has subscribed to
          if (odom_on)
          {
            sl::TRACKING_STATE tracking_error = zed.getPosition(pose);
            if(tracking_error == sl::TRACKING_STATE_SEARCHING) lost_frames++;
            else lost_frames = 0;
            //std::cerr << "SEARCHING SINCE " << lost_frames*frame_time.count() << std::endl;
            if(lost_frames*frame_time.count()>10e6 && reset)
            {
              grab_status = zed.resetTracking(sl::Transform());
              tracking_error = zed.getPosition(pose);
              pub_odom.resize(0);
              lost_frames = 0;
              std::cerr << "ZED TRACK LOST RESET POSITION" << std::endl;
            }
            if(tracking_error!=sl::TRACKING_STATE_OK) std::cout << sl::toString(tracking_error) << std::endl;
            publishOdom(pose, pub_odom, tstamp, max_buffer_length);
          }

          // Ask for a mesh update if mesh_rate has expired since last request and grab mesh
          if(map_on && mapping_activated)
          {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(time - tmesh).count()>1e3/mesh_rate)
            {
                //std::cout << "Request mesh" << std::endl;
                zed.requestMeshAsync();
                tmesh = time; // MyClock::now();
            }

            if (zed.getMeshRequestStatusAsync() == sl::SUCCESS)
            {
                zed.retrieveMeshAsync(mesh);
                mesh_extracted = true;
                //std::cout << "Mesh retrieved with " << mesh.getNumberOfTriangles() << " Triangles" << std::endl;
                //std::cout << "Mesh retrieved, current fps " << zed.getCurrentFPS() << std::endl;
            }
          }

          //store images do disk if requested
          if(bimg_on)
          {
            if (std::chrono::duration_cast<std::chrono::milliseconds>(time - tbimg).count()>1e3/bimg_rate)
            {
              zed.retrieveImage(leftZEDMat, lv);
              saveZEDMattoImage(bimg_prefix, leftZEDMat, tstamp, bimg_endfix);
              tbimg = time; //MyClock::now();
            }
          }

          //loop_time.sleep();

        } 
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // No subscribers, we just wait
        }
        if(autocalibration)
        {
          //make sure exposure/gain/brightness/hue are adjusted automatically
          zed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, 0, true);
          autocalibration = false;
        }
      } // while loop
      zed.close();
    }

  public:
    int saveMesh(std::string fname)
    {
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
      //make sure it is set
      device_poll_thread = nullptr;

      //default loop controls
      limg_on  = false;
      rimg_on  = false;
      depth_on = false;
      odom_on  = false;
      imu_on   = false;
      map_on   = false;
      mesh_extracted = false;
      autocalibration = false;
      reset = false;

      //default camera parameters
      resolution    = sl::RESOLUTION_VGA;
      quality       = sl::DEPTH_MODE_PERFORMANCE;
      sensing_mode  = sl::SENSING_MODE_STANDARD;
      mesh_rate     = 2;
      frame_rate    = 30;
      camera_rate   = frame_rate;
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
      max_buffer_length = 1;

      lv = sl::VIEW_LEFT;
      rv = sl::VIEW_RIGHT;

      bimg_on = false;
      bimg_rate = 1;
    }

    //This function is used to start the thread that will run in the background and talks to the ZED
    void startWrapperThread()
    {
      std::cout << "Initializing the wrapper server" << std::endl;
      std::cout << "   Left Image Topic  : " << (limg_on?pub_left.name():std::string("off")) << std::endl;
      std::cout << "   Right Image Topic : " << (rimg_on?pub_right.name():std::string("off")) << std::endl;
      std::cout << "   Depth Topic       : " << (depth_on?pub_depth.name():std::string("off")) << std::endl;
      std::cout << "   Odometry Topic    : " << (odom_on?pub_odom.name():std::string("off")) << std::endl;
      std::cout << "   Imu Topic         : " << (imu_on?pub_left.name():std::string("off")) << std::endl;
      std::cout << "   Mapping           : " << (map_on?"on":"off") << std::endl;
      std::cout << "   Buffer length     : " << max_buffer_length << std::endl;
      std::cout << "   Frame rate        : " << frame_rate << std::endl;
      std::cout << "   Camera rate       : " << camera_rate << std::endl;
      std::cout << "   Imu rate          : " << imu_rate << std::endl;
      std::cout << "   Mesh rate         : " << mesh_rate << std::endl;
      std::cout << "   Confidence        : " << confidence << std::endl;
      std::cout << "   Rectified         : " << ((lv == sl::VIEW_LEFT && rv == sl::VIEW_RIGHT)?"True":"False") << std::endl;

      // Try to initialize the ZED
      if (!svo_filename.empty())
          param.svo_input_filename = svo_filename.c_str();
      else
      {
        param.camera_fps = camera_rate;
        param.camera_resolution = static_cast<sl::RESOLUTION> (resolution);
        if (serial_number == 0)
          param.camera_linux_id = zed_id;
        else
        {
          bool waiting_for_camera = true;
          while (waiting_for_camera)
          {
            sl::DeviceProperties prop = getZEDFromSN(serial_number);
            if (prop.id < -1 || prop.camera_state == sl::CAMERA_STATE::CAMERA_STATE_NOT_AVAILABLE)
            {
              std::string msg = "ZED SN" + std::to_string(serial_number) + " not detected ! Please connect this ZED";
              std::cout << msg.c_str() << std::endl;
              std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            }
            else
            {
              waiting_for_camera = false;
              param.camera_linux_id = prop.id;
            }
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
      while (err != sl::SUCCESS)
      {
        err = zed.open(param);
        std::cout << toString(err) << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      }

      std::cout << "Connect ZED" << std::endl;
      std::cout << "   ZED Model                 : " << toString(zed.getCameraInformation().camera_model) << std::endl;
      std::cout << "   ZED Serial Number         : " << zed.getCameraInformation().serial_number << std::endl;
      std::cout << "   ZED Firmware              : " << zed.getCameraInformation().firmware_version << std::endl;
      std::cout << "   ZED Camera Resolution     : " << zed.getResolution().width << "x" << zed.getResolution().height << std::endl;
      std::cout << "   ZED Camera FPS            : " << zed.getCameraFPS() << std::endl;

      //make sure exposure/gain/brightness/hue are adjusted automatically
      zed.setCameraSettings(sl::CAMERA_SETTINGS_EXPOSURE, 0, true);
      autocalibration = false;

      //set confidence threshold
      zed.setConfidenceThreshold(confidence);

      serial_number = zed.getCameraInformation().serial_number;

      info_left  = fillCamInfo(zed, lv);
      info_right = fillCamInfo(zed, rv);
      imu_trans = zed.getCameraInformation().camera_imu_transform; //TODO: Marco: Seems to be a bug in the SDK


      run_wrapper = true;
      device_poll_thread = new std::thread(&ZEDWrapper::device_poll, this);
    }

    void stopWrapperThread()
    {
      run_wrapper = false;
      if(device_poll_thread)
      {
        device_poll_thread->join();
        delete device_poll_thread;
      }
      device_poll_thread = nullptr;
    }

    ~ZEDWrapper()
    {
      stopWrapperThread();
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

    void setDepthTopic(std::string name)
    {
      depth_stabilization = true;
      pub_depth.initialize(name);
      if(!pub_depth.is_owner()){ pub_depth.force_remove(); pub_depth.initialize(); }
      depth_on = true;
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
      imu_on = true;
    }

    void setLocalImageStorage(std::string prefix, double rate, std::string endfix = std::string("jpg"))
    {
      bimg_prefix = prefix;
      bimg_endfix = endfix;
      bimg_rate = rate;
      bimg_on = true;
    }


    inline void setMappingFlag(bool on)
    {
      depth_stabilization = true;
      map_on = on;
    }

    inline void setLocalImageStorageFlag(bool on)
    {
      bimg_on = on;
    }


    inline void doNotRectifyImages()
    {
      lv = sl::VIEW_LEFT_UNRECTIFIED;
      rv = sl::VIEW_RIGHT_UNRECTIFIED;
    }

    inline void doAutoCalibration()
    {
      autocalibration = true;
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

    inline void setCameraRate(double hertz)
    {
      camera_rate = hertz;
      std::cout << "Camera rate set to " << camera_rate << std::endl;
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

    inline void setSVOFile(std::string svo_file)
    {
      svo_filename = svo_file;
    }


    inline bool setReset(bool flag)
    {
      reset = flag;
      std::cout << "Reset flag set to " << reset << std::endl;
    }

    inline bool getMappingFlag()
    {
      return map_on;
    }

    inline bool getDepthFlag()
    {
      return depth_on;
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

    inline CameraInfo getLeftCamaraInfo()
    {
      return info_left;
    }

    inline CameraInfo getRightCamaraInfo()
    {
      return info_right;
    }

    void fillZEDConfiguration(ZEDContextStreamConfiguration &conf, 
      const CameraInfo &info, ZEDGridPositionAndOrientation pose, const int id)
    {
      conf.componentPositionAndOrientation = pose;
      conf.videoConfiguration.componentPositionAndOrientation = pose;
      conf.videoConfiguration.componentId = id;
      for(int i=0; i<3; i++) conf.videoConfiguration.intrinsics.K[i];
      for(int i=0; i<2; i++) conf.videoConfiguration.intrinsics.P[i];
      if(id>=3)
      {
        conf.videoConfiguration.fileName = "";
        conf.videoConfiguration.framesPerSecond = 0;
        conf.videoConfiguration.verticalResolution   = 0;
        conf.videoConfiguration.horizontalResolution = 0;
        conf.videoConfiguration.isRectified = 0;
        conf.videoConfiguration.isDeBayered = 0;
        conf.videoConfiguration.intrinsics.fx = 0;
        conf.videoConfiguration.intrinsics.cx = 0;
        conf.videoConfiguration.intrinsics.fy = 0;
        conf.videoConfiguration.intrinsics.cy = 0;
        conf.videoConfiguration.verticalFOV   = 0; 
        conf.videoConfiguration.horizontalFOV = 0;
      }

      if(id<=2)
      {
        conf.videoConfiguration.fileName = "";
        conf.videoConfiguration.framesPerSecond = frame_rate;
        conf.videoConfiguration.verticalResolution   = info.height;
        conf.videoConfiguration.horizontalResolution = info.width;
        conf.videoConfiguration.isRectified = info.rectified;
        conf.videoConfiguration.isDeBayered = true; //i think so
        double cx=info.P[2], cy=info.P[6], fx=info.P[0], fy=info.P[5];
        conf.videoConfiguration.intrinsics.fx = fx;
        conf.videoConfiguration.intrinsics.cx = cx;
        conf.videoConfiguration.intrinsics.fy = fy;
        conf.videoConfiguration.intrinsics.cy = cy;
        conf.videoConfiguration.intrinsics.K[0]=(info.D[0]);
        conf.videoConfiguration.intrinsics.K[1]=(info.D[1]);
        conf.videoConfiguration.intrinsics.K[2]=(info.D[4]);
        conf.videoConfiguration.intrinsics.P[0]=(info.D[2]);
        conf.videoConfiguration.intrinsics.P[1]=(info.D[3]);
        conf.videoConfiguration.verticalFOV   = 45/atan(1)*(atan(cy/fy) + 
                             atan((conf.videoConfiguration.verticalResolution-cy)/fy)); 
        conf.videoConfiguration.horizontalFOV = 45/atan(1)*(atan(cx/fx) + 
                             atan((conf.videoConfiguration.horizontalResolution-cx)/fx));
      }
    }

    std::vector <ZEDContextStreamDefinition> getZEDStreamDefinitions()
    {
      std::vector <ZEDContextStreamDefinition> msg_arr;

      ZEDContextStreamDefinition msg;
      msg.formatVersion = "1.0";
      msg.documentationURI = "no documentation available yet";

      msg.component.componentName = "ZEDMini";
      msg.component.vendorName    = "Stereolabs";
      msg.component.serialNumber  = std::to_string(serial_number);

      ZEDGridPositionAndOrientation pose;

      long long ts = std::chrono::duration_cast<std::chrono::milliseconds>(MyClock::now().time_since_epoch()).count();
      msg.configuration.timeStamp = ts;
      msg.configuration.videoConfiguration.timeStamp = ts;

      if(limg_on)
      {
        pose.position[0]=0;
        pose.position[1]=0;
        pose.position[2]=0;
        pose.rotation[0]=0;
        pose.rotation[1]=0;
        pose.rotation[2]=0;
        pose.rotation[3]=1;
        msg.streamFormat  = "ImageHeaderMsg:Left";
        msg.streamAddress = pub_left.name();
        fillZEDConfiguration(msg.configuration, info_left, pose, 0);
        msg_arr.push_back(msg);
      }
      if(rimg_on)
      {
        pose.position[0]=-1*info_right.P[3]/info_right.P[0];
        pose.position[1]=0;
        pose.position[2]=0;
        pose.rotation[0]=0;
        pose.rotation[1]=0;
        pose.rotation[2]=0;
        pose.rotation[3]=1;
        msg.streamFormat  = "ImageHeaderMsg:Right";
        msg.streamAddress = pub_right.name();
        fillZEDConfiguration(msg.configuration, info_right, pose, 1);
        msg_arr.push_back(msg);
      }
      if(depth_on)
      {
        pose.position[0]=0;
        pose.position[1]=0;
        pose.position[2]=0;
        pose.rotation[0]=0;
        pose.rotation[1]=0;
        pose.rotation[2]=0;
        pose.rotation[3]=1;
        msg.streamFormat  = "ImageHeaderMsg:Depth";
        msg.streamAddress = pub_depth.name();
        fillZEDConfiguration(msg.configuration, info_left, pose, 2);
        msg_arr.push_back(msg);
      }
      if(odom_on)
      {
        pose.position[0]=0;
        pose.position[1]=0;
        pose.position[2]=0;
        pose.rotation[0]=0;
        pose.rotation[1]=0;
        pose.rotation[2]=0;
        pose.rotation[3]=1;
        msg.streamFormat  = "PoseMsg";
        msg.streamAddress = pub_odom.name();
        fillZEDConfiguration(msg.configuration, info_left, pose, 3);
        msg.configuration.videoConfiguration.framesPerSecond = frame_rate;
        msg_arr.push_back(msg);
      }
      if(imu_on)
      {
        sl::Translation pos = imu_trans.getTranslation();
        pose.position[0]=pos(0);
        pose.position[1]=pos(1);
        pose.position[2]=pos(2);
        sl::Orientation rot = imu_trans.getOrientation();
        pose.rotation[0]=rot(0);
        pose.rotation[1]=rot(1);
        pose.rotation[2]=rot(2);
        pose.rotation[3]=rot(3);
        msg.streamFormat  = "ImuMsg";
        msg.streamAddress = pub_imu.name();
        fillZEDConfiguration(msg.configuration, info_left, pose, 4);
        msg.configuration.videoConfiguration.framesPerSecond = imu_rate;
        msg_arr.push_back(msg);
      }
      return msg_arr;
    }

}; // class ZEDWrapper

#endif //#ifndef ZEDWRAPPER_HPP
