///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2017, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////


/********************************************************************************
 ** This sample demonstrates how to grab images and change the camera settings **
 ** with the ZED SDK                                                           **
 ********************************************************************************/


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

// Using std and sl namespaces
using namespace std;
using namespace cv;
using namespace sl;

//HD1270
//VGA672

struct mytype{
  double member[672];
};


struct C
{
    int size;
    int cols; 
    int rows;
    int type;
    double pad[670];
};

int main(int argc, char **argv) {
  std::cout << "Connect buffer" << std::endl;
  SharedBuffer <mytype> buf("ZedImageBuffer");
  std::cout << "buffer connected as owner " << buf.is_owner() << std::endl;

  if(argc>1)
  {
    std::cout << "Reading from buffer of size " << buf.get_size() << std::endl;
    vector<mytype> outbuf;
    
    auto p=buf.read(outbuf);
    int i=0;
    while(p!=outbuf.end())
    {
      C *c=(C*)&*p;
      cv::Mat mymat(c->rows, c->cols, c->type, &*(p+sizeof(C)));

      namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
      imshow( "Display window", mymat );                   // Show our image inside it.
      std::cout << "Showing " << std::distance(outbuf.begin(),p) << std::endl;
      cv::waitKey(5);                                          // Wait for a keystroke in the window
      std::cout << "We managed here" << std::endl;
      p+=(sizeof(C)+c->size)/sizeof(mytype);
      if(p==outbuf.end()) p=buf.read(outbuf);
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

  // Create a Mat to store images
  sl::Mat zed_image;
  // Create a Mat to store images
  // Capture new images until 'q' is pressed
  char key = ' ';
  mytype *pstart, *hstart;
  C c= {0};
  int cnt=0;
  while (true) {
      // Check that grab() is successful      
      if (zed.grab(rp) == SUCCESS) {

          // Retrieve left image
          zed.retrieveImage(zed_image, VIEW_LEFT);
          c.size=(int)(zed_image.getHeight()*zed_image.getWidth()*zed_image.getPixelBytes());
          c.cols=(int) zed_image.getWidth();
          c.rows=(int) zed_image.getHeight();
          pstart= (mytype*)zed_image.getPtr<sl::uchar1>(sl::MEM_CPU);
          hstart = (mytype*)&c;
          buf.lock(true);
          buf.write(hstart, hstart+sizeof(C)/sizeof(mytype), false);
          buf.write(pstart, pstart+c.size/sizeof(mytype), false);
          buf.lock(false);
          //retrieve right image
          zed.retrieveImage(zed_image, VIEW_RIGHT);
          c.size=(int)(zed_image.getHeight()*zed_image.getWidth()*zed_image.getPixelBytes());
          c.cols=(int) zed_image.getWidth();
          c.rows=(int) zed_image.getHeight();
          c.type=CV_8UC4;
          pstart= (mytype*)zed_image.getPtr<sl::uchar1>(sl::MEM_CPU);
          hstart = (mytype*)&c;
          buf.lock(true);
          buf.write(hstart, hstart+sizeof(C)/sizeof(mytype), false);
          int len=buf.write(pstart, pstart+c.size/sizeof(mytype), false);
          buf.lock(false);
          if(cnt>100) buf.pop(2*(c.size+sizeof(C))/sizeof(mytype));
          if(cnt%30==0) std::cout << "Wrote 2x" << c.size+sizeof(C) << " Buffer length " << len << " " << zed.getCurrentFPS() << "Hz" << std::endl;
          cnt++;
      }
      sl::sleep_ms(20);
      //key = cv::waitKey(5);
  }

  
  // Exit
  zed.close();
  return EXIT_SUCCESS;
}

