/***********************************************************************************
 * Sample of how to write and read images from/to shared buffers.
 * 
 * Author:  Marco Salathe <msalathe@lbl.gov>
 * Date:    Mai 2018
 * License: If you like to use this code, please contact the author.
 *
 **********************************************************************************/

#include "SimpleBuffer.hpp"
#include "MsgDefinition.hpp"

 // Standard includes
#include <iostream>
#include <string.h>
#include <ctime>
#include <thread>
#include <signal.h>

// OpenCV include (for display)
//#include "opencv2/opencv.hpp"
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
  std::cout << "Connect buffer" << std::endl;
  SimpleBuffer <ImageHeaderMsg> buf("ImageBuffer");
  std::cout << "buffer connected as owner " << buf.is_owner() << std::endl;
  long long last_time = 0, time_sum = 0, time_cnt=0;

  if(buf.is_owner() == false)
  {
    std::cout << "Reading from buffer of size " << buf.size() << std::endl;
    int i=0, key;
    while(key != 'q' && stop == false)
    {
      ImageHeaderMsg* hdr = buf.read_simple(20, 10, 1000);
      if(hdr == nullptr) break;
      cv::Mat mymat(hdr->rows, hdr->cols, hdr->type, hdr->data);
      imshow( "Display window", mymat );                //prepare the image in a window
      key = cv::waitKey(5);                             //show the image and wait for a keystroke in the window

      //just a bit of output
      long long time = hdr->time.sec*1e6 + hdr->time.usec; 
      long long delta = time - last_time;
      if(last_time==0){ delta=0; }
      last_time = time;
      time_sum+=delta; time_cnt++;
      if(time_sum>1e6){
        std::cout << "Extracted " << time_cnt << " images of " << hdr->cols << "x" << hdr->rows << " (" << hdr->size << "bytes, type "
                << hdr->type << ") at " << 1e6*time_cnt/time_sum << "Hz" << std::endl; 
        time_sum=0; time_cnt=0;
      }
    }
    return 0;
  }

  cv::Mat image;                                        // Create a Mat to store images to
  image = cv::imread("../myimage.jpg");                 // Read image from file
  if(! image.data )                                     // Check for invalid input
  {
    std::cout <<  "ERROR: Could not open or find the image" << std::endl ;
    return -1;
  }

  //Create the image header
  ImageHeaderMsg hdr;
  hdr.size       = (int)(image.total()*image.elemSize());
  hdr.block_size = (int)(sizeof(ImageHeaderMsg));
  hdr.block_nmb  = (int)(hdr.size/hdr.block_size+1.0); //add a block if required
  hdr.cols       = (int)(image.cols);
  hdr.rows       = (int)(image.rows);
  hdr.type       = image.type();

  std::cout << "Total image size " << hdr.size << " blocks " << hdr.block_nmb << " blocksize " << hdr.block_size << std::endl;
  
   //Make sure if the hdr.size doesn't fit sizeof(ImageHeaderMsg) perfectly
   //we do not exceed the memory and cause a segfault
  int elm_on_row = hdr.size/image.rows;
  image.reserve(hdr.block_size*hdr.block_nmb/elm_on_row+1);
  hdr.data      = (void*)image.data;


  while(stop == false)
  {
    //calculate accurate time stamps
    std::chrono::system_clock::time_point stime = std::chrono::system_clock::now();
    std::chrono::seconds const sec = std::chrono::duration_cast<std::chrono::seconds>(stime.time_since_epoch());
    hdr.time.sec  = sec.count();
    hdr.time.usec = std::chrono::duration_cast<std::chrono::microseconds>(stime.time_since_epoch() - sec).count();

    //write the image to the buffer until Ctrl+C is pressed
    buf.write_simple(hdr, 100);
    std::this_thread::sleep_for(std::chrono::milliseconds(10)); //don't write faster than 100Hz

    //just a bit of output
    long long time = hdr.time.sec*1e6 + hdr.time.usec; 
    long long delta = time - last_time;
    if(last_time==0){ delta=0; }
    last_time = time;
    time_sum+=delta; time_cnt++;
    if(time_sum>1e6){
      std::cout << "Wrote " << time_cnt << " images of " << hdr.cols << "x" << hdr.rows << " (" << hdr.size << "bytes, type "
              << hdr.type << ") at " << 1e6*time_cnt/time_sum << "Hz" << std::endl; 
      time_sum=0; time_cnt=0;
    }
  }
  return 0;
}

