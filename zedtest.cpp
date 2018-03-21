#define USE_MANAGED_SHARED_MEMORY

 // Standard includes
#include <iostream>
#include <string>
#include <chrono>

// ZED include
//#include <boost/interprocess/containers/vector.hpp>
#include <sl_zed/Camera.hpp>

// OpenCV include (for display)
//#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SharedBuffer.hpp"
#include "ZEDWrapper.hpp"

// Using std and sl namespaces
using namespace std;
using namespace cv;
//using namespace sl;

int main(int argc, char **argv) {
  std::chrono::system_clock::time_point last_time=std::chrono::system_clock::now();
  long long time_sum=0;
  int time_cnt=0;
  if(argc==1){ std::cout << "need buffer address" << std::endl; return 0; }

  if(argc>2)
  {
    std::string tmp(argv[2]);
    if(tmp.compare("image")==0)
    {
      std::cerr << "Connect to image buffer" << std::endl;
      SharedBuffer <ImageHeaderMsg> buf(argv[1]);
      std::cerr << "buffer connected as owner " << buf.is_owner() << std::endl;

      char key = ' ';
      while(key != 'q')
      {
        vector<ImageHeaderMsg> outbuf;
        //std::cout << "Reading from buffer of size " << buf.size() << std::endl;
        auto p=buf.read(outbuf);
        while(p!=outbuf.end())
        {
          ImageHeaderMsg *c=(ImageHeaderMsg*)&*p;
          p++;
          //std::cout << "Showing " << c->size << ", " << c->cols << "x" << c->rows << ", " << c->type << std::endl;
          cv::Mat mymat(c->rows, c->cols, c->type, &*(p));


          //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
          imshow( "Display window", mymat );                   // Show our image inside it.
          key = cv::waitKey(5);                                          // Wait for a keystroke in the window
          std::chrono::system_clock::time_point time = std::chrono::system_clock::now();
          long long delta = std::chrono::duration_cast<std::chrono::microseconds>(time - last_time).count();
          last_time=time;
          time_sum+=delta; time_cnt++;
          if(time_sum>1e6){ std::cout << "Extracted images at " << 1e6*time_cnt/time_sum << "Hz" << std::endl; time_sum=0; time_cnt=0; };
          p+=(c->size)/sizeof(ImageHeaderMsg);
        }
      }
      return 0;
    }
    else if(tmp.compare("odom")==0)
    {
      std::cerr << "Connect to odom buffer" << std::endl;
      SharedBuffer <PoseMsg> buf2(argv[1]);
      std::cerr << "buffer connected as owner " << buf2.is_owner() << std::endl;
      //char key = ' ';
      while(buf2.is_owner() == false)
      {
        vector<PoseMsg> outbuf;
        //std::cout << "Reading from buffer of size " << buf.size() << std::endl;
        auto p=buf2.read(outbuf);
        while(p!=outbuf.end())
        {
          //std::chrono::system_clock::time_point time = std::chrono::system_clock::now();
          //long long delta = std::chrono::duration_cast<std::chrono::microseconds>(time - last_time).count();
          //last_time=time;
          //time_sum+=delta; time_cnt++;
          //if(time_sum>1e6){ 
          //std::cout << "Extracted pose at " << 1e6*time_cnt/time_sum << "Hz. " ;
          std::cout << p->time.sec << "." << p->time.usec << " " << p->position[0] << " " << p->position[1] << " " << p->position[2] << std::endl;
          //time_sum=0; time_cnt=0; 
          //};
          p++;
        }
        //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
        //imshow( "Display window", mymat );                   // Show our image inside it.
        //key = cv::waitKey(5);                                          // Wait for a keystroke in the window
      }
      return 0;
    }
  }

  std::cerr << "Connect buffer" << std::endl;
  SharedBuffer <ImageHeaderMsg> buf3(argv[1]);
  std::cerr << "buffer connected as owner " << buf3.is_owner() << std::endl;  
  
  // Create a ZED Camera object
  sl::Camera zed;

  sl::InitParameters par;
  par.camera_resolution=sl::RESOLUTION_VGA;
  par.camera_fps =30;
  par.depth_mode = sl::DEPTH_MODE_NONE;
  sl::RuntimeParameters rp;
  rp.enable_depth=false;
  rp.enable_point_cloud=false;

  // Open the camera
  sl::ERROR_CODE err = zed.open(par);
  if (err != sl::SUCCESS) {
    cout << toString(err) << endl;
    zed.close();
    return 1; // Quit if an error occurred
  }


  // Print camera information
  printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
  printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
  printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
  printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
  printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

  sl::Mat zed_image;
  ImageHeaderMsg *pstart;
  ImageHeaderMsg c = {0};
  int cnt=0;
  while (true) {
    if (zed.grab(rp) == sl::SUCCESS) {
      // Retrieve image
      zed.retrieveImage(zed_image, sl::VIEW_SIDE_BY_SIDE);
      c.size=(int)(zed_image.getHeight()*zed_image.getWidth()*zed_image.getPixelBytes());
      c.cols=(int) zed_image.getWidth();
      c.rows=(int) zed_image.getHeight();
      c.type=CV_8UC4;

      pstart= (ImageHeaderMsg*)zed_image.getPtr<sl::uchar1>(sl::MEM_CPU);

      int len = buf3.write({ &c, &c+1, pstart, pstart+c.size/sizeof(ImageHeaderMsg) });
      int pops=0;
      if(cnt>100) pops=buf3.resize(900*(c.size/sizeof(ImageHeaderMsg)+1));

      if(cnt%30==0) std::cout << "Wrote " << c.size+sizeof(ImageHeaderMsg) << " Buffer length " << len << "-" << pops << " " << zed.getCurrentFPS() << "Hz" << std::endl;
      cnt++;
    }
    sl::sleep_ms(20);
  }

  
  // Exit
  zed.close();
  return EXIT_SUCCESS;
}

