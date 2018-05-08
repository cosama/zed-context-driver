/***********************************************************************************
 * Header file that defines the SimpleBuffer class, a simpler way to use shared
 * SharedBuffers,
 * 
 * Author:  Marco Salathe <msalathe@lbl.gov>
 * Date:    Mai 2018
 * License: If you like to use this code, please contact the author.
 *
 **********************************************************************************/
#include "SharedBuffer.hpp"
#include "MsgDefinition.hpp"

#include <chrono>
#include <thread>
#include <iostream>

template <class T> class SimpleBuffer: public SharedBuffer<T>
{
  private:
    std::vector<T> buffer;
    typename std::vector<T>::iterator iter;

  public:
    SimpleBuffer<T>(): SharedBuffer<T>() { iter = buffer.end(); };
    SimpleBuffer(std::string bname, int bsize = 65536): SharedBuffer<T>(bname, bsize) { iter = buffer.end(); };

    //A function that can be used to read a fixed sized structure/class (return) from the buffer, ms_sleep indicates 
    //how many milliseconds it should sleep if the buffer is empty before trying to grab a gain and timeout is the time in 
    //milliseconds after that it returns NULL. If timeout is 0 then it waits indefinitely. 'number' tells the function how 
    //many elements it should try to grab maximally (it is better to grab multiple, as the remaining images might get cleared 
    //from the buffer any time). If 'number' is 0 then it grabs all available data (if the buffer is large this might cause a 
    //memory error).
    T* read_simple(int number, int ms_sleep, int timeout)
    {
      int cnt=0;
      if(iter==buffer.end()){ SharedBuffer<T>::lock(); buffer.clear(); iter=SharedBuffer<T>::read(buffer, number); }
      while(iter==buffer.end() && (timeout==0 || cnt*ms_sleep<timeout))
      {
        SharedBuffer<T>::unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(ms_sleep)); 
        SharedBuffer<T>::lock();
        iter=SharedBuffer<T>::read(buffer, number);
        cnt++;
      }
      SharedBuffer<T>::unlock();
      if(iter==buffer.end()){ return nullptr; }
      T *hdr=(T*)&*iter;             //create a pointer to the header of the image (first image block)
      iter++;                        //move the iterator to the begining of the image data
      return hdr;
    };

    //A function that can be used to write a fixed sized structure/class 'data' to the buffer. 
    //'max_data' defines the maximum number of images that are stored in the buffer. Once the buffer 
    //exceeds that number it will be resized to that size.
    int write_simple(T &data, int max_data)
    {
      SharedBuffer<T>::lock();
      int buf_size = SharedBuffer<T>::write(data);
      while(max_data>0 && buf_size>max_data)
      {
        SharedBuffer<T>::resize(max_data);
        buf_size = SharedBuffer<T>::size();
      }
      SharedBuffer<T>::unlock();
      return buf_size;
    };
};

//A function that can be used to write a image 'hdr' to the buffer. The hdr.data
//needs to point to the memory block (continous) where the image data are stored. 'max_img'
//defines the maximum number of images that are stored in the buffer. Once the buffer exceeds
//that number it will be resized to that size.
template<> ImageHeaderMsg* SimpleBuffer<ImageHeaderMsg>::read_simple(int number, int ms_sleep, int timeout)
{
  ImageHeaderMsg* hdr;
  int cnt=0;

  if(iter==buffer.end()){ SharedBuffer<ImageHeaderMsg>::lock(); buffer.clear(); iter=SharedBuffer<ImageHeaderMsg>::read(buffer, (number?1:0)); }
  while(iter==buffer.end() && (timeout==0 || cnt*ms_sleep<timeout))
  {
    SharedBuffer<ImageHeaderMsg>::unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(ms_sleep)); 
    SharedBuffer<ImageHeaderMsg>::lock();
    iter=SharedBuffer<ImageHeaderMsg>::read(buffer, (number?1:0));
    cnt++;
  }
  if(iter==buffer.end()){ SharedBuffer<ImageHeaderMsg>::unlock(); return nullptr; }
  hdr=(ImageHeaderMsg*)&*iter;             //create a pointer to the header of the image (first image block)
  iter++;                                  //move the iterator to the begining of the image data
  if(iter==buffer.end())
  { 
    iter=SharedBuffer<ImageHeaderMsg>::read(buffer, ((number>0)?(number*(hdr->block_nmb+1)-1):0)); 
    hdr=(ImageHeaderMsg*)&*(iter-1); //previous stuff might reallocate vector thus we have to link it again
  }
  SharedBuffer<ImageHeaderMsg>::unlock(); //we always unlock, if already unlocked nothing really will happen
  hdr->data=(void*)&*iter;
  iter+=hdr->block_nmb;
  return hdr;
};

//A function that can be used to write a image 'hdr' to the buffer. The hdr.data
//needs to point to the memory block (continous) where the image data are stored. 'max_img'
//defines the maximum number of images that are stored in the buffer. Once the buffer exceeds
//that number it will be resized to that size.
template<> int SimpleBuffer<ImageHeaderMsg>::write_simple(ImageHeaderMsg &hdr, int max_img)
{
  ImageHeaderMsg* img_ptr= (ImageHeaderMsg*)hdr.data;
  int img_len=hdr.block_nmb;
  if(!hdr.data) img_len=0;
  SharedBuffer<ImageHeaderMsg>::lock(); //we need the lock anyways, this way nobody can access the buffer before the resize
  int buf_size = SharedBuffer<ImageHeaderMsg>::write({ &hdr, &hdr+1, img_ptr, img_ptr+img_len });
  int cur_img = get_user_info();
  if(buf_size == 0) cur_img = 0; else if(buf_size == img_len+1) cur_img = 1; else cur_img++;
  while(max_img>0 && cur_img>max_img)
  {
    std::vector <ImageHeaderMsg> buf_tmp;
    SharedBuffer<ImageHeaderMsg>::read(buf_tmp, 1);
    if(buf_tmp.size()==1)
    {
      ImageHeaderMsg* hdr_tmp = (ImageHeaderMsg*)&buf_tmp.front();
      SharedBuffer<ImageHeaderMsg>::resize(buf_size-(hdr_tmp->block_nmb+1)); //internally makes sure we don't pass zero
      cur_img--;
    }
    buf_size = SharedBuffer<ImageHeaderMsg>::size();
    if(buf_size == 0) cur_img = 0;
  }
  set_user_info(cur_img);
  SharedBuffer<ImageHeaderMsg>::unlock();
  return cur_img;
};
