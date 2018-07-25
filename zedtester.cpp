/***********************************************************************************
 * Small script to test shared buffers
 * 
 * Author:  Marco Salathe <msalathe@lbl.gov>
 * Date:    July 2018
 * License: See License.txt in parent folder of this repository.
 *
 **********************************************************************************/

//compiler flag for shared memory or shared file
//#define USE_MANAGED_SHARED_MEMORY

// ZED Wrapper (for messages) and shared buffer includes
#include "SimpleBuffer.hpp"
#include "MsgDefinition.hpp"

// Standard includes
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <signal.h>

// ZED SDK include
#include <sl_zed/Camera.hpp>

// OpenCV include (for display)
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Using std and sl namespaces
using namespace std;
using namespace cv;

//we need to make sure the constructor is called on exit (Ctrl+C) otherwise
//the mutex might get stuck in a looked state
bool stop = false;
void handler(int) {
    std::cout << "Will exit" << std::endl;
    stop = true;
}

int main(int argc, char **argv) {

  signal(SIGINT, &handler); //initiate the signal handler, so that on Ctrl+C the buffer does not get stuck
  std::chrono::system_clock::time_point last_time=std::chrono::system_clock::now();
  long long last_buf=0;

  if(argc==1){ std::cout << "Need the buffer address as second argument" << std::endl; return 0; }

  if(argc>2)
  {
    std::string tmp(argv[2]);

    //Example of how to read and display images from the shared buffer
    if(tmp.compare("image")==0)
    {
      long long time_sum=0, buf_sum;
      int time_cnt=0;
      std::cerr << "Connect to image buffer" << std::endl;

      //Connect to a buffer containing images (ImageHeaderMsg defines both the header and block size, one image is composed of multiple blocks)
      SimpleBuffer <ImageHeaderMsg> buf(argv[1]);

      //the owner of the buffer creates and cleans up the shared buffer and usually writes to the buffer
      //it should be false (0) if another process is writting and owning the buffer
      std::cerr << "buffer connected as owner " << buf.is_owner() << " " << buf.size() << std::endl;

      //read the pictures until 'q' is pressed (the opencv window must be open and selected for this to work)
      char key = ' ';
      vector<ImageHeaderMsg> outbuf;               //a STL vector to read out all ImageheaderMsg blocks currently stored to the buffer
      std::vector<ImageHeaderMsg>::iterator iter;
      ImageHeaderMsg *c=NULL;
      int cnt = 0;
      while(key != 'q' && stop == false)
      {
        ImageHeaderMsg* c = buf.read_simple(20, 10, 1000);
        if(c == nullptr) break;
        cv::Mat mymat(c->rows, c->cols, c->type, c->data);

        if(argc>3)
        {
          std::string jpgname = std::to_string(cnt);
          jpgname =std::string(argv[3]) + std::string(6 - jpgname.length(), '0') + jpgname + ".jpg";
          imwrite(jpgname, mymat);
          cnt++;
        }
        imshow( "Display window", mymat );                //prepare the image in a window
        key = cv::waitKey(20);                            //show the image and wait for a keystroke in the window

        //the rest is to print the image retrieval frequency and image data to the terminal
        std::chrono::system_clock::time_point time = std::chrono::system_clock::now();
        long long buf_time = c->time.sec*1e6 + c->time.usec; 
        long long rdelta = std::chrono::duration_cast<std::chrono::microseconds>(time - last_time).count();
        long long bdelta = buf_time - last_buf;
        if(last_buf==0){ bdelta=0; }
        last_time=time;
        last_buf =buf_time;
        time_sum+=rdelta; buf_sum+=bdelta; time_cnt++;
        if(time_sum>1e6){
        std::cout << "Extracted an image of " << c->cols << "x" << c->rows << " (" << c->size << "bytes, type "
                  << c-> type << ") at real " << 1e6*time_cnt/time_sum << "Hz buffer " << 1e6*time_cnt/buf_sum << "Hz" << std::endl; 
          time_sum=0; buf_sum=0; time_cnt=0;
        };
        //}
      }
      return 0;
    }
    
    //Example of how to read and use POSE data from shared buffer
    else if(tmp.compare("odom")==0)
    {
      std::cerr << "Connect to odometry buffer" << std::endl;

      //Connect to a buffer containing PoseMsg blocks
      SimpleBuffer <PoseMsg> buf2(argv[1]);

      //the owner of the buffer creates and cleans up the shared buffer and usually writes to the buffer
      //it should be false (0) if another process is writting and owning the buffer
      std::cerr << "buffer connected as owner " << buf2.is_owner() << std::endl;

      //here we use the trick that once this process becomes the owner of the buffer no other process
      //is connected to it anymore and we can stop reading from it and clearing up and terminate
      while(buf2.is_owner() == false && stop == false)
      {
        PoseMsg *p = buf2.read_simple(20, 10, 1000);
        if(p == nullptr) break;
        std::cout << p->time.sec << "." << setfill('0') << setw(6) << p->time.usec << setfill(' ') << setw(0) << " " << p->position[0] << " " << p->position[1] << " " << p->position[2] << " " 
          << p->orientation[0] << " " << p->orientation[1] << " " << p->orientation[2] << " " << p->orientation[3] << std::endl;
      }
      return 0;
    }
  }

  //Example on how to send pictures to the buffer (with the ZED SDK)
  std::cerr << "Create image buffer" << std::endl;

  //create a buffer containing images (ImageHeaderMsg defines both the header and block size, one image is composed of multiple blocks)
  SimpleBuffer <ImageHeaderMsg> buf3(argv[1]);

  //the owner of the buffer creates and cleans up the shared buffer and usually writes to the buffer
  //it should be true (1) in this case as the process is writting and owning the buffer
  std::cerr << "buffer connected as owner " << buf3.is_owner() << std::endl;  
  
  //Create a ZED Camera object (see the SDK documentation for more information
  //https://www.stereolabs.com/developers/documentation/API/v2.3.3/ )
  sl::Camera zed;

  //some camera parameters
  sl::InitParameters par;
  par.camera_resolution=sl::RESOLUTION_VGA;
  par.camera_fps =30;
  par.depth_mode = sl::DEPTH_MODE_NONE;

  //open the camera and print an error message if the camera can not be opened
  sl::ERROR_CODE err = zed.open(par);
  if (err != sl::SUCCESS) {
    cout << toString(err) << endl;
    zed.close();
    return 1; // quit if an error occurred
  }

  //print camera information
  printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
  printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
  printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
  printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
  printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

  sl::Mat zed_image;                   //Create a ZED SDK image matrix (similar to OpenCV image matrices)
  ImageHeaderMsg c = {0};              //And an empty image header
  int cnt=0;
  while (stop == false) {              //Stop only on Ctrl+C
    if (zed.grab() == sl::SUCCESS) {   //Check if the ZED SDK has a new image available

      //retrieve the image and fill it to the matrix
      zed.retrieveImage(zed_image, sl::VIEW_SIDE_BY_SIDE);

      //fill the header with the image data
      c.size=(int)(zed_image.getHeight()*zed_image.getWidth()*zed_image.getPixelBytes());
      c.cols=(int) zed_image.getWidth();
      c.rows=(int) zed_image.getHeight();
      c.type=CV_8UC4;
      c.block_size = (int)(sizeof(ImageHeaderMsg));
      c.block_nmb  = (int)(c.size/c.block_size);

      //put the pointer to the start of the image block
      c.data = (void*)zed_image.getPtr<sl::uchar1>(sl::MEM_CPU);

      //write the header and the image data to the buffer
      int len = buf3.write_simple(c, 100);

      //print some information about what was just stored to the buffer
      if(cnt%30==0) std::cout << "Wrote " << c.size+sizeof(ImageHeaderMsg) << " Buffer length " << len << " " << zed.getCurrentFPS() << "Hz" << std::endl;
      cnt++;
    }
    sl::sleep_ms(20); //the SDK has it's own sleep function
  }
  zed.close(); //close the camera and exit
  return EXIT_SUCCESS;
}

