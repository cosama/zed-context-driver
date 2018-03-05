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
#include <sl_zed/Camera.hpp>

// OpenCV include (for display)
#include "opencv2/opencv.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

// Sample functions
//void updateCameraSettings(char key, sl::Camera &zed);
//void switchCameraSettings();

// Sample variables
CAMERA_SETTINGS camera_settings_ = CAMERA_SETTINGS_BRIGHTNESS;
string str_camera_settings = "BRIGHTNESS";
int step_camera_setting = 1;


int main(int argc, char **argv) {

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
    printHelp();

    // Print camera information
    printf("ZED Model                 : %s\n", toString(zed.getCameraInformation().camera_model).c_str());
    printf("ZED Serial Number         : %d\n", zed.getCameraInformation().serial_number);
    printf("ZED Firmware              : %d\n", zed.getCameraInformation().firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) zed.getResolution().width, (int) zed.getResolution().height);
    printf("ZED Camera FPS            : %d\n", (int) zed.getCameraFPS());

    // Create a Mat to store images
    Mat zed_image;
    SharedBuffer <Mat> buf("ZedImageBuffer");
    
    // Capture new images until 'q' is pressed
    char key = ' ';
    while (key != 'q') {
        if(buf.owner())
        {
          // Check that grab() is successful
          if (zed.grab() == SUCCESS) {
              // Retrieve left image
              zed.retrieveImage(zed_image, VIEW_LEFT);
              buf.write(zed_image,10);

              // Display image with OpenCV
              //cv::imshow("VIEW", cv::Mat((int) zed_image.getHeight(), (int) zed_image.getWidth(), CV_8UC4, zed_image.getPtr<sl::uchar1>(sl::MEM_CPU)));
              //key = cv::waitKey(5);

              // Change camera settings with keyboard
              //updateCameraSettings(key, zed);
          } else
              key = cv::waitKey(5);
        }
        else
        {
            vector<Mat>outbuf = buf.read();
            for(int i=1; i<outbuf.size(); i++)
              cv::imshow("VIEW", cv::Mat((int) outbuf[i].getHeight(), (int) outbuf[i].getWidth(), CV_8UC4, outbuf[i].getPtr<sl::uchar1>(sl::MEM_CPU)));
        }
    }

    // Exit
    zed.close();
    return EXIT_SUCCESS;
}

