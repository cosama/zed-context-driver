/***********************************************************************************
 * Simple class template to share a buffer between processes/threads. Any standard
 * type and fixed sized structure/class can be used, as long as memory is not
 * dynamically allocate (STL containers do not work because of this).
 *
 * Instances can add elements individually by calling the write member function. 
 * The buffer is usually read in chunks calling the read member function and is returned
 * as a vector (stored to local memory). The memory is locked on read/write thus other
 * element will have no or restricted access at this point. If the destructor of the
 * Shared buffers owner instance is called it will clear and remove the buffer. If 
 * the destructor is not called (i.e Ctrl+C) the shared_memory will stay in memory. 
 * If a process is terminated while it locks the buffer, no other buffer can access
 * the buffer without lifting the lock. In both cases an instance has to connect to 
 * the memory and call force_remove() to lift the lock (if present) and remove the 
 * buffer (in linux the same result can be achieved by removing the respective file
 * in /dev/shm). If all processes are terminated properly then none of this should ever
 * be an issue. The buffer keeps track of how many elements have been already read
 * by a given instance and only provides new elements on later reads. It also deals
 * with memory allocation and expands the memory if required.
 *
 * Author:  Marco Salathe <msalathe@lbl.gov>
 * Date:    March 2018
 * License: See License.txt in parent folder of this repository.
 *
 * WHISHLIST:
 *   * Everything is managed manually through stat, we should just add mutex there
 *   * STL containers
 *   * Can the SharedMemory pointer go out of scope?
 *
 **********************************************************************************/

#ifndef SHAREDBUFFER_HPP
#define SHAREDBUFFER_HPP

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_sharable_mutex.hpp>
#include <boost/container/scoped_allocator.hpp>
#include <string>
#include <vector>
//#include <iostream>

#ifdef USE_MANAGED_SHARED_MEMORY
typedef boost::interprocess::managed_shared_memory SharedMemory;
#else
#include <boost/interprocess/managed_mapped_file.hpp>
typedef boost::interprocess::managed_mapped_file SharedMemory;
#endif

template <class T> using SharedAllocator =  boost::container::scoped_allocator_adaptor<boost::interprocess::allocator<T, SharedMemory::segment_manager> >;

template <class T> using BufferContainer = boost::interprocess::deque<T, SharedAllocator<T> >;

#define MAX_CLIENTS 1024

class BufferStatus{
  public:
  BufferStatus(bool b, int alloc_size)
  { 
    alive=b; mem_size = alloc_size; pop_size=MAX_CLIENTS; 
    for(int i=0; i<pop_size; i ++) pop_counter[i]=-1; 
  }
  bool alive;
  long long mem_size;
  int pop_size;
  int user_info;
  long long pop_counter[MAX_CLIENTS];
};

template <class T> class SharedBuffer
{
  private:
    BufferContainer<T> *buffer;
    bool owner;
    bool locked;
    int id;
    int alloc_size;
    int csize;
    std::string mem_name;
    std::string mut_name;
    std::string sta_name;
    SharedMemory *memory;
    boost::interprocess::named_sharable_mutex *nm;
    boost::interprocess::managed_shared_memory *stamem;
    BufferStatus *stat;

  public:
    //Initialize or reinitialize the function, mostly a copy of the constructor
    //do be used with the standard constructor, which can not initizialize properly.
    //returns 0 if error occurs, otherwise returns 1
    int initialize(std::string bname = "", int bsize = 0)
    {
      //bring memory into a acceptable state
      owner      = false;
      locked     = false;
      buffer     = NULL;
      stat       = NULL;
      if(nm!=NULL) delete nm;
      if(memory!=NULL) delete memory;
      if(stamem!=NULL) delete stamem;

      //parse inputs and decide what to do
      mem_name.assign(bname);
      std::string msname=bname;
      if(!bname.empty())
      {
        #ifdef _WIN32
        size_t i = bname.rfind('\\', bname.length());
        #else
        size_t i = bname.rfind('/', bname.length());
        #endif
        if (i != std::string::npos) {
            msname.assign(bname.substr(i+1, bname.length() - i));
        }
        mut_name.assign(msname+"Mut");
        sta_name.assign(msname+"Sta");
      }
      if(bname.empty() || mem_name.empty()) return 0; //no valid name, allows directory, caught by boost but not gracefully
      if(bsize==0)
      {
        if(alloc_size==0) alloc_size = 1024*1024; //1Mb
      }
      else alloc_size = bsize;

      //create lock
      nm = new boost::interprocess::named_sharable_mutex(boost::interprocess::open_or_create, mut_name.c_str());

      //create a new segment with given name and size
      memory = new SharedMemory(boost::interprocess::open_or_create, mem_name.c_str(), alloc_size);

      //create status handler
      stamem = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, sta_name.c_str(), 4096+sizeof(BufferStatus));
      stat = stamem->find_or_construct<BufferStatus>(sta_name.c_str())(true, alloc_size);
      stat->alive=true;
      csize = stat->mem_size;

      int i=0;
      for(i=0; i<stat->pop_size; i++)
        if(stat->pop_counter[i]<0){ id=i; stat->pop_counter[i]=0; break; }
      if(i==stat->pop_size) id=-1; //invalid

      //try to find the buffer with the same name as the momory block if not found it returns NULL (0)
      buffer = memory->find<BufferContainer<T> >(mem_name.c_str()).first;

      //if the buffer is not find, we are considered the owner of this shared memory and will construct the buffer
      if(!buffer)
      {
        //initialize shared memory STL-compatible allocator
        const SharedAllocator<T> alloc(memory->get_segment_manager());

        //construct a T deque buffer
        buffer = memory->construct<BufferContainer<T> >(mem_name.c_str())(SharedAllocator<T>(memory->get_segment_manager()));
        owner = true;
      }
      return 1;
    };

    //Destroy shared_memory, we need to recreate it with initialize after this call if
    //the instance is the owner he might loss ownership if another process recreates the
    //buffer before the owner can do so.
    void force_remove()
    {
      if(stat) stat->alive = false;
      //we do not remove the objects as they will be destroyed anyways once the memory is freed
      //also destroying them caused errors as we would have to check if they exist.
      #ifdef USE_MANAGED_SHARED_MEMORY
      boost::interprocess::shared_memory_object::remove(mem_name.c_str());
      #else
      //boost::interprocess::file_mapping::remove(mem_name.c_str());
      #endif
      boost::interprocess::named_sharable_mutex::remove(mut_name.c_str());
      boost::interprocess::shared_memory_object::remove(sta_name.c_str());
      if(nm!=NULL)     delete nm;
      if(memory!=NULL) delete memory;
      if(stamem!=NULL) delete stamem;
      id         = -1;
      alloc_size = 0;
      owner      = false;
      locked     = false;
      buffer     = NULL;
      stat       = NULL;
      memory     = NULL;
      stamem     = NULL;
      nm         = NULL;
    };

    void check_alive()
    {
      if(stat && stat->alive==false)
        initialize();
    }

    void check_mem()
    {
      if(stat)
      {
        if(stat->mem_size!=csize)
        {
          csize=stat->mem_size;
          delete memory;
          memory = new SharedMemory(boost::interprocess::open_only, mem_name.c_str());
          buffer = memory->find<BufferContainer<T> >(mem_name.c_str()).first;
        }
      }
    }

    SharedBuffer<T>()
    {
      id         = -1;
      alloc_size = 0;
      owner      = false;
      locked     = false;
      buffer     = NULL;
      stat       = NULL;
      memory     = NULL;
      stamem     = NULL;
      nm         = NULL;
    }

    SharedBuffer<T>(std::string bname, int bsize = 65536)
    {
      id         = -1;
      alloc_size = 0;
      owner      = false;
      locked     = false;
      buffer     = NULL;
      stat       = NULL;
      memory     = NULL;
      stamem     = NULL;
      nm         = NULL;
      initialize(bname, bsize);
    }

    virtual ~SharedBuffer<T>()
    {
      if(locked && nm!=NULL) nm->unlock();
      locked = false;
      if(stat && id >= 0 && id <= stat->pop_size) stat->pop_counter[id]=-1; //unregister
      if(owner == true) force_remove();
      if(nm != NULL) delete nm;
      if(memory != NULL) delete memory;
      if(stamem != NULL) delete stamem;
      id     = -1;
      stat   = NULL;
      buffer = NULL;
    };

    //Writes one element to the buffer. Returns the size of the buffer
    int write(T &obj)
    {
      check_alive();
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      check_mem(); //reconnect or recreate
      while(true)
      {
        try
        {
          buffer->push_back(obj);
        }
        catch(boost::interprocess::bad_alloc)
        {
          if(stat) stat->mem_size+=alloc_size;
          memory->grow(mem_name.c_str(), alloc_size);
          check_mem();
          continue;
        }
        break;
      }
      return buffer->size();
    };

    //Writes elements from iterator first to last to the buffer. Returns the size of the buffer.
    template <class InpIt> 
    int write(InpIt first, InpIt last)
    {
      check_alive();
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      check_mem(); //reconnect or recreate
      //int bsize=buffer->size();
      while(true)
      {
        try
        {
          buffer->insert(buffer->end(), first, last);
        }
        catch(boost::interprocess::bad_alloc)
        {
          if(stat) stat->mem_size+=alloc_size;
          memory->grow(mem_name.c_str(), alloc_size);
          check_mem();
          continue;
        }
        break;
      }
      return buffer->size();
    };

    //Writes elements from iterator itl[0] to itl[1], itl[2] to til[3], etc to buffer. Returns 
    //the size of the buffer. This can write a whole list of elements to the buffer.
    template <class InpIt> 
    int write(std::initializer_list<InpIt> itl)
    {
      check_alive();
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      check_mem(); //reconnect or recreate
      //int bsize=buffer->size();
      for(auto it=itl.begin(); it!=itl.end() && (it+1)!=itl.end(); it+=2)
      {
        while(true)
        {
          try
          {
            buffer->insert(buffer->end(), *it, *(it+1));
          }
          catch(boost::interprocess::bad_alloc)
          {
            if(stat) stat->mem_size+=alloc_size;
            memory->grow(mem_name.c_str(), alloc_size);
            check_mem();
            continue;
          }
          break;
        }
      }
      return buffer->size();
    };

    //Read the elements that have been added to the buffers since the last read and adds it to 'vec'.
    //It returns an iterator to vec that points to the initial end of the vector
    typename std::vector<T>::iterator read(std::vector<T> &vec, int max_len=0)
    {
      check_alive();
      if(buffer==NULL) return vec.end(); //can not read
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> sharable_lock(*nm);
      check_mem(); //reconnect or recreate
      int bsize=buffer->size();
      if(max_len<=0 || bsize<max_len) max_len=bsize; 
      auto start = buffer->begin();
      if(stat && id>=0 && id<stat->pop_size)
      {
        if(stat->pop_counter[id]>=bsize) return vec.end();
        if(stat->pop_counter[id]+max_len>=bsize) max_len=bsize-stat->pop_counter[id];
        if(stat->pop_counter[id]>0)
        {
          start+=stat->pop_counter[id];
        }
        stat->pop_counter[id] += max_len;
      }

      auto end = start + max_len; //less than end...
      int oldpos = vec.size();
      vec.insert(vec.end(), start , end);
      return vec.begin()+oldpos;
    };

    //Removes a certain number of elements from the buffer
    int resize(unsigned int new_size)
    {
      check_alive();
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      check_mem(); //reconnect or recreate
      int pops=buffer->size()-new_size;
      if(pops<=0) return 0; //nothing to be done
      auto start = buffer->begin();
      buffer->erase(start, start+pops);
      if(stat) for(int i = 0; i < stat->pop_size; i++)
      { 
        if(stat->pop_counter[i]>0)
        { 
          stat->pop_counter[i]-=pops;
          if(stat->pop_counter[i]<0) stat->pop_counter[i]=0;
        }
      }
      return pops;
    }

    //Check if this instance is the owner of the memory.
    inline bool is_owner()
    { 
      return owner;
    };
    
    //Changes the ownership to 'state'
    inline void set_owner(bool state)
    { 
      owner = state;
    };

    //Get the buffer size, use this with care, as it can be always changed by writing
    //new elements into the buffer
    inline int size()
    { 
      if(buffer!=NULL)
        return buffer->size();
      else return -1;
    };

    //Get the shared buffer name
    inline std::string name()
    { 
      return mem_name;
    };

    //Aquire an exclusive lock, any operation, writing and reading is possible after that
    //but no other thread can access the buffer until released again with unlock(). 
    inline void lock()
    {
      if(!locked && nm!=NULL) nm->lock();
      locked = true;
    };

    //Release the execlusive lock. After that other threads can normally access the buffer.
    //This function can be called even if the buffer has not been locked yet.
    inline void unlock()
    {
      if(locked){ if(nm!=NULL) nm->unlock(); locked = false; }
    };

    //A shared buffer has a user info attached (int) that can be use to pass messages
    //here we set the message value
    inline void set_user_info(int info)
    {
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      if(stat) stat->user_info = info;
    };

    //A shared buffer has a user info attached (int) that can be use to pass messages
    //here we get the message value
    int get_user_info()
    {
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> sharable_lock(*nm);
      if(stat) return stat->user_info;
      return 0;
    };
    
    //Assigns values from first to last to the buffer overwriting any previously insert elements and
    //shrinking the buffer in size or expanding it to fit first to last
    template<typename InpIt> int assign(InpIt first, InpIt last)
    {
      check_alive();
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      if(!locked && nm!=NULL) boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      check_mem(); //reconnect or recreate
      while(true)
      {
        try
        {
          buffer->assign(first, last);
        }
        catch(boost::interprocess::bad_alloc)
        {
          if(stat) stat->mem_size+=alloc_size;
          memory->grow(mem_name.c_str(), alloc_size);
          check_mem();
          continue;
        }
        break;
      }
      return buffer->size();
    };

};

#endif //#ifndef SHAREDBUFFER_HPP
