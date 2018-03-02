#include <SharedBuffer.hpp>
#include <iostream>
#include <unistd.h>

int main(int argc, char *argv[])
{
  //boost::interprocess::shared_memory_object::remove("VisualBuffer");
  //boost::interprocess::named_mutex::remove("VisualBufferMut");
  SharedBuffer <int> buf("VisualBuffer");
  std::cout << "We managed to connect/create to the buffer, owner " << buf.is_owner() << std::endl;

  if(argv[1])
  {
    buf.force_remove();
    return 0;
  }
  if(buf.is_owner()){
    int i=0;
    while(true){  //Insert data in the vector
       buf.write(i,100000);
       i++;
       if(i>200000) i=0;
    }
  }
  else{
    while(true){
      std::vector<int> v=buf.read();
      for(int i=1; i<v.size(); i++)
        if(v[i]-v[i-1]!=1) std::cout << "Read " << v[i] << " " << v[i-1] << " at " << i << std::endl;
      //for(auto it = v.begin()+1; it!=v.end(); ++it)  //Insert data in the vector
      //  if(*it-*(it-1)!=1) std::cout << "Read " << *it-*(it-1) << std::endl;
    }
  }

  return 0;
}


