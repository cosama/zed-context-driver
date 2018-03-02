#ifndef SHAREDBUFFER_HPP
#define SHAREDBUFFER_HPP

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <string>
#include <vector>
#include <iostream>

/***********************************************************************************
 * Simple class template to share a buffer between processes/threads.
 *
 * Instances can add elements individually by calling the write member function, 
 * however, the whole buffer needs to be read in one go and is returned as a 
 * vector (stored to local memory). The memory is locked on read/write thus no 
 * other element will have access at this point. If the owner-process is 
 * terminated without calling the destructor the shared_memory might hang around. 
 * In that case a instance has to connect to the memory and call force_remove().
 *
 * Author: Marco Salathe <msalathe@lbl.gov>
 * Date:   February 2018
 *
 **********************************************************************************/

template <class T> class SharedBuffer{
  private:
    boost::interprocess::deque<T, boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>> *buffer;
    bool owner;
    std::string mem_name;
    std::string que_name;
    std::string mut_name;
    boost::interprocess::managed_shared_memory *memory;
    boost::interprocess::named_mutex *nm;
    int size;

  public:
    SharedBuffer<T>(std::string bname, int bsize=65536): mem_name(bname), que_name(bname+"Que"), mut_name(bname+"Mut") {
      //assign remaining private variables
      size= bsize;
      owner = false;
      buffer = NULL;

      nm = new boost::interprocess::named_mutex(boost::interprocess::open_or_create, mut_name.c_str());
      //bool look_ok=boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*nm); //not really necessary to lock here

      //Create a new segment with given name and size
      memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, mem_name.c_str(), size);

      //try to find the buffer with the same name as the momory block if not found it returns NULL (0)
      buffer = memory->find<boost::interprocess::deque<T, boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>(que_name.c_str()).first;

      //if the buffer is not find, we are considered the owner of this shared memory and will construct the buffer
      
      if(!buffer)
      {
        //initialize shared memory STL-compatible allocator
        const boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> alloc(memory->get_segment_manager());
        //construct a T deque buffer
        buffer = memory->construct<boost::interprocess::deque<T, boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>(que_name.c_str())(alloc);
        owner = true;
      }
    }


    ~SharedBuffer<T>(){
      if(owner == true){
        memory->destroy<T>(que_name.c_str());
        boost::interprocess::shared_memory_object::remove(mem_name.c_str());
        boost::interprocess::named_mutex::remove(mut_name.c_str());
      }
      if(nm!=NULL)     delete nm;
      if(memory!=NULL) delete memory;
    };

    //writes to buffer, if buffer size is larger than max_length remove elements
    //so that max length is assured, if max_length is zero nothing is done
    int write(T &obj, int max_length=0)
    {
      boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*nm);
      while(true)
      {
        try{
          buffer->push_back(obj);
        }
        catch(boost::interprocess::bad_alloc){
            memory->grow(mem_name.c_str(), size);
            delete memory;
            memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, mem_name.c_str(), size);
            buffer = memory->find<boost::interprocess::deque<T, boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>(que_name.c_str()).first;
            continue;
        }
        break;
      }
      if(max_length>0) while(buffer->size()>max_length) buffer->pop_front();
      return 0;
    };

    //reads the entire buffer and return
    std::vector<T> read()
    {
      boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*nm);
      return std::vector<T> (buffer->begin(), buffer->end());;
    };
    
    bool is_owner(){ 
      return owner;
    }
    
    void force_remove(){
      memory->destroy<T>(que_name.c_str());
      boost::interprocess::shared_memory_object::remove(mem_name.c_str());
      boost::interprocess::named_mutex::remove(mut_name.c_str());
      delete nm;
      delete memory;
      memory = NULL;
      nm = NULL;
      owner = false;
      size = 0;
    }
};

#endif //#ifndef SHAREDBUFFER_HPP
