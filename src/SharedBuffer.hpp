#ifndef SHAREDBUFFER_HPP
#define SHAREDBUFFER_HPP

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/named_sharable_mutex.hpp>
#include <boost/container/scoped_allocator.hpp>
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
 * be an issue. The buffer keeps track of how many elements have been already read
 * by a given instance and only reads new elements on later reads.
 *
 * Author: Marco Salathe <msalathe@lbl.gov>
 * Date:   March 2018
 *
 **********************************************************************************/

template <class T> using SharedAllocator =  boost::container::scoped_allocator_adaptor<boost::interprocess::allocator<T, boost::interprocess::managed_shared_memory::segment_manager> >;

template <class T> using BufferContainer = boost::interprocess::deque<T, SharedAllocator<T> >;

#define MAX_CLIENTS 1024

class BufferStatus{
  public:
  BufferStatus(bool b){ alive=b; size=MAX_CLIENTS; for(int i=0; i<size; i ++) pop_counter[i]=-1; };
  bool alive;
  int size;
  long long pop_counter[MAX_CLIENTS];
};

template <class T> class SharedBuffer
{
  private:
    BufferContainer<T> *buffer;
    bool owner;
    int id;
    int size;
    std::string mem_name;
    std::string que_name;
    std::string mut_name;
    std::string sta_name;
    boost::interprocess::managed_shared_memory *memory;
    boost::interprocess::named_sharable_mutex *nm;
    BufferStatus *stat;
  public:
    SharedBuffer<T>()
    {
      id     = -1;
      size   = 0;
      owner  = false;
      buffer = NULL;
      stat   = NULL;
      memory = NULL;
      nm     = NULL;
    }

    SharedBuffer<T>(std::string bname, int bsize = 65536): mem_name(bname), que_name(bname+"Que"), mut_name(bname+"Mut"), sta_name(bname+"Sta")
    {
      //assign remaining private variables
      size   = bsize;
      owner  = false;
      buffer = NULL;
      stat   = NULL;

      nm = new boost::interprocess::named_sharable_mutex(boost::interprocess::open_or_create, mut_name.c_str());
      //bool look_ok=boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm); //not really necessary to lock here

      //Create a new segment with given name and size
      memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, mem_name.c_str(), size);

      stat = memory->find_or_construct<BufferStatus>(sta_name.c_str())(true); //(SharedAllocator<T>(memory->get_segment_manager()));
      stat->alive=true;
      int i=0;
      for(i=0; i<stat->size; i++)
        if(stat->pop_counter[i]<0){ id=i; stat->pop_counter[i]=0; break; }
      if(i==stat->size) id=-1; //invalid

      //try to find the buffer with the same name as the momory block if not found it returns NULL (0)
      buffer = memory->find<BufferContainer<T> >(que_name.c_str()).first;

      //if the buffer is not find, we are considered the owner of this shared memory and will construct the buffer
      if(!buffer)
      {
        //initialize shared memory STL-compatible allocator
        buffer = memory->construct<BufferContainer<T> >(que_name.c_str())(SharedAllocator<T>(memory->get_segment_manager()));
        owner  = true;
      }
    }

    ~SharedBuffer<T>()
    {
      if(stat && id>=0 && id<=stat->size) stat->pop_counter[id]=-1; //unregister
      if(owner == true)
      {
        if(stat) stat->alive = false;
        memory->destroy<T>(que_name.c_str());
        boost::interprocess::shared_memory_object::remove(mem_name.c_str());
        boost::interprocess::named_sharable_mutex::remove(mut_name.c_str());
      }
      if(nm!=NULL)     delete nm;
      if(memory!=NULL) delete memory;
      id     = -1;
      stat   = NULL;
      buffer = NULL;
    };

    //Initialize or reinitialize the function, mostly a copy of the constructor
    //do be used with the standard constructor, which can not initizialize properly.
    void initialize(std::string bname, int bsize = 65536)
    {
      mem_name.assign(bname);
      que_name.assign(bname+"Que");
      mut_name.assign(bname+"Mut");
      sta_name.assign(bname+"Sta");
      size   = bsize;
      owner  = false;
      buffer = NULL;
      stat   = NULL;

      if(nm!=NULL) delete nm;
      if(memory!=NULL) delete memory;

      nm = new boost::interprocess::named_sharable_mutex(boost::interprocess::open_or_create, mut_name.c_str());
      //bool look_ok=boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm); //not really necessary to lock here

      //create a new segment with given name and size
      memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, mem_name.c_str(), size);

      stat = memory->find_or_construct<BufferStatus>(sta_name.c_str())(true); //(SharedAllocator<T>(memory->get_segment_manager()));
      stat->alive=true;
      int i=0;
      for(i=0; i<stat->size; i++)
        if(stat->pop_counter[i]<0){ id=i; stat->pop_counter[i]=0; break; }
      if(i==stat->size) id=-1; //invalid

      //try to find the buffer with the same name as the momory block if not found it returns NULL (0)
      buffer = memory->find<BufferContainer<T> >(que_name.c_str()).first;

      //if the buffer is not find, we are considered the owner of this shared memory and will construct the buffer
      if(!buffer)
      {
        //initialize shared memory STL-compatible allocator
        const SharedAllocator<T> alloc(memory->get_segment_manager());
        //construct a T deque buffer
        buffer = memory->construct<BufferContainer<T> >(que_name.c_str())(SharedAllocator<T>(memory->get_segment_manager()));
        owner = true;
      }
    };

    //Writes one element to the buffer. Returns the size of the buffer
    int write(T &obj)
    {
      if(stat && stat->alive==false) initialize(mem_name, size); //reconnect or recreate
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      while(true)
      {
        try
        {
          buffer->push_back(obj);//, SharedAllocator<T>(memory->get_segment_manager()));
        }
        catch(boost::interprocess::bad_alloc)
        {
          memory->grow(mem_name.c_str(), size);
          delete memory;
          memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only, mem_name.c_str());
          stat = memory->find<BufferStatus>(sta_name.c_str()).first;
          buffer = memory->find<BufferContainer<T> >(que_name.c_str()).first;
          continue;
        }
        break;
      }
      return buffer->size();
    };

    //Writes elements from first to last to the buffer. Returns the size of the buffer.
    template <class InpIt> 
    int write(InpIt first, InpIt last, bool lock=true)
    {
      if(stat && stat->alive==false) initialize(mem_name, size); //reconnect or recreate
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      //boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      if(lock) nm->lock();
      while(true)
      {
        try
        {
          buffer->insert(buffer->end(), first, last);
        }
        catch(boost::interprocess::bad_alloc)
        {
          memory->grow(mem_name.c_str(), size);
          delete memory;
          memory = new boost::interprocess::managed_shared_memory(boost::interprocess::open_only, mem_name.c_str());
          stat = memory->find<BufferStatus>(sta_name.c_str()).first;
          buffer = memory->find<BufferContainer<T> >(que_name.c_str()).first;
          continue;
        }
        break;
      }
      if(lock) nm->unlock();
      return buffer->size();
    };



    //Read the elements that have been added to the buffers since the last read and adds it to vec.
    //It returns an iterator to vec with the last position
    typename std::vector<T>::iterator read(std::vector<T> &vec)
    {
      if(stat && stat->alive==false) initialize(mem_name, size); //reconnect or recreate
      if(buffer==NULL) return vec.end(); //can not read
      boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> sharable_lock(*nm);
      auto start = buffer->begin();
      auto end = buffer->end();
      int bsize=buffer->size();
      if(stat && id>=0 && id<stat->size)
      { 
        if(stat->pop_counter[id]<=bsize && stat->pop_counter[id]>0)
          start+=stat->pop_counter[id];
        stat->pop_counter[id]=buffer->size();
      }
      int oldpos=vec.size();
      if(start!=end) vec.insert(vec.end(), start , end);
      return vec.begin()+oldpos;
    };

    //removes a certain number of elements from the buffer
    int pop(unsigned int num_of_elements)
    {
      if(num_of_elements<=0) return 0;
      if(stat && stat->alive==false) initialize(mem_name, size); //reconnect or recreate
      if(buffer==NULL) return 0; //can not write must be wrongly initiated could try to reinitiate
      boost::interprocess::scoped_lock<boost::interprocess::named_sharable_mutex> lock(*nm);
      auto start = buffer->begin();
      int pops=buffer->size()<num_of_elements?buffer->size():num_of_elements;
      buffer->erase(start, start+pops);
      if(stat) for(int i = 0; i < stat->size; i++)
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
    bool is_owner()
    { 
      return owner;
    };
    
    //Changes the ownership from false to true or from true to false
    void flip_owner()
    { 
      owner = ((owner==true)?false:true);
    };

    //Changes the ownership from false to true or from true to false
    void lock(bool lock)
    { 
      if(lock) nm->lock();
      else nm->unlock();
    };

    //Get the buffer size, use this with care, as it can be always changed by writing
    //new elements into the buffer
    int get_size()
    { 
      return buffer->size();
    };

    //Destroy shared_memory, we need to recreate it with initialize after this call if
    //the instance is the owner he might loss ownership if another process recreates the
    //buffer before the owner can do so.
    void force_remove()
    {
      if(stat) stat->alive = false;
      memory->destroy<T>(que_name.c_str());
      boost::interprocess::shared_memory_object::remove(mem_name.c_str());
      boost::interprocess::named_sharable_mutex::remove(mut_name.c_str());
      if(nm!=NULL)     delete nm;
      if(memory!=NULL) delete memory;
      id     = -1;
      memory = NULL;
      nm     = NULL;
      buffer = NULL;
      stat   = NULL;
      owner  = false;
      size   = 0;
    };
};

#endif //#ifndef SHAREDBUFFER_HPP
