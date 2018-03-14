#define USE_MANAGED_SHARED_MEMORY

 // Standard includes
#include <iostream>
#include <string.h>

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
using namespace sl;

int main(int argc, char **argv) {

  if(argc==1){ std::cout << "need buffer address" << std::endl; return 0; }

  std::cout << "Connect buffer" << std::endl;
  SharedBuffer <ImageHeader> buf(argv[1]);
  std::cout << "buffer connected as owner " << buf.is_owner() << std::endl;

  if(argc>2)
  {

    char key = ' ';
    while(key != 'q')
    {
      vector<ImageHeader> outbuf;
      //std::cout << "Reading from buffer of size " << buf.size() << std::endl;
      auto p=buf.read(outbuf);
      while(p!=outbuf.end())
      {
        ImageHeader *c=(ImageHeader*)&*p;
        p++;
        //std::cout << "Showing " << c->size << ", " << c->cols << "x" << c->rows << ", " << c->type << std::endl;
        cv::Mat mymat(c->rows, c->cols, c->type, &*(p));

        //namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
        imshow( "Display window", mymat );                   // Show our image inside it.
        key = cv::waitKey(5);                                          // Wait for a keystroke in the window

        //std::cout << "We managed here" << std::endl;
        p+=(c->size)/sizeof(ImageHeader);
      }
    }
    return 0;
  }


  // Create a ZED Camera object
  Camera zed;

  sl::InitParameters par;
  par.camera_resolution=sl::RESOLUTION_VGA;
  par.camera_fps =30;
  par.depth_mode = sl::DEPTH_MODE_NONE;
  sl::RuntimeParameters rp;
  rp.enable_depth=false;
  rp.enable_point_cloud=false;

  // Open the camera
  ERROR_CODE err = zed.open(par);
  if (err != SUCCESS) {
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
  ImageHeader *pstart;
  ImageHeader c = {0};
  int cnt=0;
  while (true) {
    if (zed.grab(rp) == SUCCESS) {
          // Retrieve image
          zed.retrieveImage(zed_image, VIEW_SIDE_BY_SIDE);
          c.size=(int)(zed_image.getHeight()*zed_image.getWidth()*zed_image.getPixelBytes());
          c.cols=(int) zed_image.getWidth();
          c.rows=(int) zed_image.getHeight();
          c.type=CV_8UC4;

          pstart= (ImageHeader*)zed_image.getPtr<sl::uchar1>(sl::MEM_CPU);

          int len = buf.write({ &c, &c+1, pstart, pstart+c.size/sizeof(ImageHeader) });
          int pops=0;
          if(cnt>100) pops=buf.resize(900*(c.size/sizeof(ImageHeader)+1));

          if(cnt%30==0) std::cout << "Wrote " << c.size+sizeof(ImageHeader) << " Buffer length " << len << "-" << pops << " " << zed.getCurrentFPS() << "Hz" << std::endl;
          cnt++;
      }
      sl::sleep_ms(20);
  }

  
  // Exit
  zed.close();
  return EXIT_SUCCESS;
}

