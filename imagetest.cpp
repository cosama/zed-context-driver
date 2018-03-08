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
  SharedBuffer <char> buf("ZedImageBuffer");
  std::cout << "buffer connected as owner " << buf.is_owner() << std::endl;

  if(argc>1)
  {
    std::cout << "Reading from buffer of size " << buf.get_size() << std::endl;
    vector<char> outbuf;
    
    auto p=buf.read(outbuf);
    int i=0;
    while(p!=outbuf.end())
    {
      C *c=(C*)&*p;
      cv::Mat mymat(c->rows, c->cols, c->type, &*(p+sizeof(C)));

      namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
      imshow( "Display window", mymat );                   // Show our image inside it.
      std::cout << "Showing " << std::distance(outbuf.begin(),p) << std::endl;
      cv::waitKey(500);                                          // Wait for a keystroke in the window
      std::cout << "We managed here" << std::endl;
      p+=sizeof(C)+c->size;
      if(p==outbuf.end()) p=buf.read(outbuf);
    }
    buf.flip_owner();
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
  auto pstart= (char*)zed_image.data;
  auto hstart = (char*)&c;
  buf.write(hstart, hstart+sizeof(C));
  buf.write(pstart, pstart+c.size);
  buf.write(hstart, hstart+sizeof(C));
  int cnt = buf.write(pstart, pstart+c.size);
  std::cout << "Wrote " << cnt << " elements to buffer" << std::endl;
  buf.flip_owner();
  return 0;
}

