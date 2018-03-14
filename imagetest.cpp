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
#include <boost/interprocess/containers/vector.hpp>
//#include <sl_zed/Camera.hpp>

// OpenCV include (for display)
//#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "SharedBuffer.hpp"
#include <ctime>

// Using std and sl namespaces
using namespace std;
using namespace cv;


struct C
{
    int size;
    int cols; 
    int rows;
    int type;
};

int main(int argc, char **argv) {
  std::cout << "Connect buffer" << std::endl;
  SharedBuffer <double> buf("ZedImageBuffer");
  std::cout << "buffer connected as owner " << buf.is_owner() << std::endl;

  if(argc>1)
  {
    std::cout << "Reading from buffer of size " << buf.size() << std::endl;

    int i=0, key;
    while(key != 'q')
    {
      vector<double> outbuf;
      buf.read(outbuf);
      auto p =outbuf.begin();
      std::cout << "Printing " << outbuf.size() << std::endl;
      if(outbuf.size()>0){
        C *c=(C*)&*p;
        std::cout << "Printing " << outbuf.size()*sizeof(double)/c->size << " " << c->size << " of " << c->cols << "x" << c->rows << std::endl;
        imshow( "Display window", cv::Mat(c->rows, c->cols, c->type, &*(p+sizeof(C)/sizeof(double))));                   // Show our image inside it.
      }
      key = cv::waitKey(5);                                          // Wait for a keystroke in the window

    }
    //buf.flip_owner();
    return 0;
  }

  // Create a Mat to store images
  cv::Mat zed_image;
  zed_image = cv::imread("../bike.jpg", CV_LOAD_IMAGE_COLOR);
  if(! zed_image.data )                              // Check for invalid input
  {
    std::cout <<  "Could not open or find the image" << std::endl ;
    return -1;
  }

  C c = {(int)(zed_image.total()*zed_image.elemSize()), zed_image.cols, zed_image.rows, zed_image.type()};
  auto pstart= (double*)zed_image.data;
  auto hstart = (double*)&c;
  std::clock_t begin = std::clock();
  int loops=0;
  std::cout << "Sizes " << sizeof(C)/sizeof(double) << " " << c.size/sizeof(double) << std::endl;
  while('q'!=cv::waitKey(5))
  {
    int cnt=buf.write({hstart, hstart+sizeof(C)/sizeof(double), pstart, pstart+c.size/sizeof(double)});
    if(loops>10) buf.resize(10*(sizeof(C)/sizeof(double)+c.size/sizeof(double)));
    std::clock_t end = std::clock();
    std::cout << "Wrote " << cnt << " elements to buffer at " <<  double(end - begin) / CLOCKS_PER_SEC << "sec" << std::endl;
    begin=end;
    loops++;
  }
  //buf.flip_owner();
  return 0;
}

