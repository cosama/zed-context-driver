#ifndef SHAREDBUFFER_HPP
#define SHAREDBUFFER_HPP

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_sharable_mutex.hpp>
#include <scoped_allocator>
#include <string>
#include <vector>
#include <iostream>

/***********************************************************************************
 * Simple class template to share a buffer between processes/threads.
 *
 * Instances can add elements individually by calling the write member function, 
 * however, the whole buffer needs to be read in one go and is returned as a 
 * vector (stored to local memory). The memory is locked on read/write thus no 
 * other element will have access at this point. If the owner-process is terminated 
 * without calling the destructor (i.e Ctrl+C) the shared_memory might hang around. 
 * If a process is terminated while it locks the buffer, no other buffer can access
 * the buffer without lifting the lock. In both cases a instance has to connect to 
 * the memory and call force_remove() to lift the lock (if present) and remove the 
 * buffer. If all processes are terminated properly then none of this should ever
 * be an issue.
 *
 * Author: Marco Salathe <msalathe@lbl.gov>
 * Date:   February 2018
 *
 **********************************************************************************/

template <class T> class SharedBuffer
{
  private:
    boost::interprocess::deque<T, std::scoped_allocator_adaptor<boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>> *buffer;
    bool *alive;
    bool owner;
    int size;
    std::string mem_name;
    std::string que_name;
    std::string mut_name;
    std::string alv_name;
    boost::interprocess::managed_shared_memory *memory;
    boost::interprocess::named_sharable_mutex *nm;

  public:
    SharedBuffer<T>()
    {
      size   = 0;
      owner  = false;
      buffer = NULL;
      alive  = NULL;
      memory = NULL;
      nm     = NULL;
    }

    SharedBuffer<T>(std::string bname, int bsize = 65536): mem_name(bname), que_name(bname+"Que"), mut_name(bname+"Mut"), alv_name(bname+"Alv")
    {
      //assign remaining private variables
      size   = bsize;
      owner  = false;
      buffer = NULL;
      alive  = NULL;

      nm = new boost::interprocess::named_sharable_mutex(boost::interprocess::open_or_create, mut_name.c_str());
      //bool look_ok=boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm); //not really necessary to lock here

      //Create a new segment with given name and size
      memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, mem_name.c_str(), size);

      alive = memory->find_or_construct<bool>(alv_name.c_str())(true);

      //try to find the buffer with the same name as the momory block if not found it returns NULL (0)
      buffer = memory->find<boost::interprocess::deque<T, std::scoped_allocator_adaptor<boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>>(que_name.c_str()).first;

      //if the buffer is not find, we are considered the owner of this shared memory and will construct the buffer
      if(!buffer)
      {
        //initialize shared memory STL-compatible allocator
        const boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> alloc(memory->get_segment_manager());
        //construct a T deque buffer
        buffer = memory->construct<boost::interprocess::deque<T, std::scoped_allocator_adaptor<boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>>(que_name.c_str())(alloc);
        owner  = true;
      }
    }

    ~SharedBuffer<T>()
    {
      if(owner == true)
      {
        //nm->sharable_lock();
        //nm->try_unlock_sharable_and_lock();
        //nm->try_unlock_and_lock();
        //nm->try_unlock_sharable();
        //nm->try_unlock();
        //boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> try_lock(*nm);
        if(alive) *alive = false;
        memory->destroy<T>(que_name.c_str());
        boost::interprocess::shared_memory_object::remove(mem_name.c_str());
        boost::interprocess::named_sharable_mutex::remove(mut_name.c_str());
      }
      if(nm!=NULL)     delete nm;
      if(memory!=NULL) delete memory;
      alive  = NULL;
      buffer = NULL;
    };

    //Initialize or reinitialize the function, mostly a copy of the constructor
    //do be used with the standard constructor, which can not initizialize properly.
    void initialize(std::string bname, int bsize = 65536)
    {
      mem_name.assign(bname);
      que_name.assign(bname+"Que");
      mut_name.assign(bname+"Mut");
      alv_name.assign(bname+"Alv");
      size   = bsize;
      owner  = false;
      buffer = NULL;
      alive  = NULL;

      if(nm!=NULL) delete nm;
      if(memory!=NULL) delete memory;

      nm = new boost::interprocess::named_sharable_mutex(boost::interprocess::open_or_create, mut_name.c_str());
      //bool look_ok=boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm); //not really necessary to lock here

      //create a new segment with given name and size
      memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, mem_name.c_str(), size);

      alive = memory->find_or_construct<bool>(alv_name.c_str())(true);

      //try to find the buffer with the same name as the momory block if not found it returns NULL (0)
      buffer = memory->find<boost::interprocess::deque<T, std::scoped_allocator_adaptor<boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>>(que_name.c_str()).first;

      //if the buffer is not find, we are considered the owner of this shared memory and will construct the buffer
      if(!buffer)
      {
        //initialize shared memory STL-compatible allocator
        const boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> alloc(memory->get_segment_manager());
        //construct a T deque buffer
        buffer = memory->construct<boost::interprocess::deque<T, std::scoped_allocator_adaptor<boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>>(que_name.c_str())(alloc);
        owner = true;
      }
    };

    //Write to buffer, if buffer size is larger than max_length remove elments until length
    //of buffer is =< max_length. If max_length is zero then elements are not removed.
    //It returns 0 on error and 1+number of popped elements on write.
    int write(T &obj, int max_length = 0)
    {
      if(alive && *alive==false) initialize(mem_name, size); //reconnect or recreate
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      while(true)
      {
        try
        {
          buffer->push_back(obj);
        }
        catch(boost::interprocess::bad_alloc)
        {
            memory->grow(mem_name.c_str(), size);
            delete memory;
            memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, mem_name.c_str(), size);
            alive  = memory->find_or_construct<bool>(alv_name.c_str())(true);
            buffer = memory->find<boost::interprocess::deque<T, std::scoped_allocator_adaptor<boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager>>>>(que_name.c_str()).first;
            continue;
        }
        break;
      }
      int stat = 1;
      if(max_length>0) while(buffer->size()>max_length){ buffer->pop_front(); stat++; }
      return stat;
    };

    //Read the entire buffer and return it, does return an empty vector if buffer is
    //empty or an error occured.
    std::vector<T> read()
    {
      if(alive && *alive==false) initialize(mem_name, size); //reconnect or recreate
      if(buffer==NULL) return std::vector<T>(); //can not read
      boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> sharable_lock(*nm);
      std::cout << buffer->size() << " " << buffer[0].size() << std::endl;
      return std::vector<T> (buffer->begin(), buffer->end());;
    };

    //Check if this instance is the owner of the memory.
    bool is_owner()
    { 
      return owner;
    };

    //Destroy shared_memory, we need to recreate it with initialize after this call if
    //the instance is the owner he might loss ownership if another process recreates the
    //buffer before the owner can do so.
    void force_remove()
    {
      //nm->sharable_lock();
      //nm->try_unlock_sharable_and_lock();
      //nm->unlock(); //take ownership
      //boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> try_lock(*nm);
      if(alive) *alive = false;
      memory->destroy<T>(que_name.c_str());
      boost::interprocess::shared_memory_object::remove(mem_name.c_str());
      boost::interprocess::named_sharable_mutex::remove(mut_name.c_str());
      delete nm;
      delete memory;
      memory = NULL;
      nm     = NULL;
      buffer = NULL;
      alive  = NULL;
      owner  = false;
      size   = 0;
    };
};

#endif //#ifndef SHAREDBUFFER_HPP
