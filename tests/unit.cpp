#include "calib/core.hpp"
#include <cmath>
#include <iostream>
#include <limits>
namespace {
void expect(bool value,const char* message) { if(!value) throw std::runtime_error(message); }
void near(double actual,double expected) { expect(std::abs(actual-expected)<1e-14,"time mismatch"); }
template<class F> void fails(F&& operation) { bool failed=false; try { operation(); } catch(const calib::Error&) { failed=true; } expect(failed,"expected contextual error"); }
}
int main() {
  using namespace calib;
  try {
    constexpr std::int64_t epoch=1700000000000000000;
    expect(parse_timestamp("9223372036854775807","test")==std::numeric_limits<std::int64_t>::max(),"int64 max");
    for(const auto* invalid:{"","-1","+1","1.0"," 1","9223372036854775808"}) fails([&]{parse_timestamp(invalid,"test");});
    near(relative_seconds(epoch+1,epoch),1e-9);
    near(relative_seconds(epoch,epoch+1),-1e-9);
    fails([]{relative_seconds(-1,0);});
    near(corner_time(epoch+1000000,epoch,100,2e-5,.003),.006);
    near(corner_time(epoch+1000000,epoch,100,-2e-5,-.003),-.004);
    near(corner_time(epoch+1000000,epoch,2000,0,.003),.004);
    near(corner_time(epoch+1,epoch,0,2e-5,0),1e-9);
    fails([&]{corner_time(epoch,epoch,std::numeric_limits<double>::quiet_NaN(),0,0);});
    fails([&]{corner_time(epoch,epoch,2,std::numeric_limits<double>::max(),0);});
    Detector detector(std::string("tag36h11"));
    for(std::uint32_t width=1;width<8;++width) for(std::uint32_t height=1;height<8;++height) {
      GrayImage blank{{width,height,1,8},std::vector<std::uint8_t>(width*height,255)};
      expect(detector.detect(blank).empty(),"tiny blank image must yield no tags");
    }
    GrayImage too_wide{{32768,4,1,8},std::vector<std::uint8_t>(32768*4,255)};
    fails([&]{detector.detect(too_wide);});
    std::cout<<"Timestamp precision and signed row timing passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
