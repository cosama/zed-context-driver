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
#include <stdio.h>
#include <string.h>

// ZED include
#include <boost/interprocess/containers/vector.hpp>
#include <sl_zed/Camera.hpp>

// OpenCV include (for display)
#include "opencv2/opencv.hpp"

#include "SharedBuffer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

int main(int argc, char **argv) {
  cout << "Connect buffer" << endl;
  SharedBuffer <boost::interprocess::vector<sl::uchar1>> buf("ZedImageBuffer");
  cout << "buffer connected" << endl;

    if(argc>1)
    {
      char key = ' ';
      while (key != 'q') {
              cout << "Start reading" << endl;
              vector<boost::interprocess::vector<sl::uchar1>>outbuf = buf.read();
              cout << "Buffer length " << outbuf.size() << endl;
              for(int i=1; i<outbuf.size(); i++)
              {
                cout << "   " << outbuf[i].size() << endl;
                //cv::imshow("VIEW", cv::Mat((int) outbuf[i].getHeight(), (int) outbuf[i].getWidth(), CV_8UC4, outbuf[i].getPtr<sl::uchar1>(sl::MEM_CPU)));
              }
              key = cv::waitKey(5);
          }
          return 0;
    }

    //buf.force_remove();
    //buf.initialize("ZedImageBuffer3");

    // Create a ZED Camera object
    Camera zed;

    // Open the camera
    ERROR_CODE err = zed.open();
    if (err != SUCCESS) {
        cout << toString(err) << endl;
        zed.close();
        return 1; // Quit if an error occurred
    }

    // Print help in console
    //printHelp();

    // Print camera information
    printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

    
    //return 0;
    // Create a Mat to store images
    Mat zed_image;

    
    // Capture new images until 'q' is pressed
    char key = ' ';
    while (key != 'q') {
        // Check that grab() is successful
        if (zed.grab() == SUCCESS) {
            // Retrieve left image
            zed.retrieveImage(zed_image, VIEW_LEFT);
            sl::uchar1 *pstart=zed_image.getPtr<sl::uchar1>(MEM_CPU);
            //sl::uchar4 *pend=zed_image.getPtr<sl::uchar4>(MEM_CPU);
            boost::interprocess::vector<sl::uchar1> imvec(pstart, pstart+zed_image.getStepBytes()*zed_image.getHeight());
            //cv::imshow("VIEW", cv::Mat((int)zed_image.getHeight(), (int)zed_image.getWidth(), CV_8UC4, imvec.data()));
            //key = cv::waitKey(5);
            cout << zed_image.getInfos().c_str() << endl;
            buf.write(imvec,10);
            cout << "Wrote" << endl;
        } else
            key = cv::waitKey(5);
    }

    // Exit
    zed.close();
    return EXIT_SUCCESS;
}

