#include "calib/core.hpp"
#include <iostream>
int main(int argc,char** argv) {
  if(argc!=2) return 2;
  try {
    auto image=calib::decode_png(argv[1],64*1024*1024);
    std::cout.write(reinterpret_cast<const char*>(image.pixels.data()),static_cast<std::streamsize>(image.pixels.size()));
    return 0;
  } catch(const calib::Error& error) { std::cerr<<error.what()<<'\n'; return error.code; }
}
