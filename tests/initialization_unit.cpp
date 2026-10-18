#include "calib/initialization.hpp"
#include <iostream>

namespace {
void require(bool value) { if(!value) throw std::runtime_error("Sampler invariant failed"); }
template<class F> void fails(F f) {
  bool failed=false; try { f(); } catch(const calib::Error&) { failed=true; } require(failed);
}
}
int main() {
  try {
    calib::InitialViewSampler sampler(50,10);
    for(std::int64_t i=0;i<50;++i) {
      calib::FrameObservations frame;
      frame.camera="cam0"; frame.timestamp_ns=1700000000000000001+i;
      frame.corners.resize(static_cast<std::size_t>(12+i%5));
      sampler.consider(std::move(frame));
    }
    const auto selected=sampler.selected(); require(selected.size()==10);
    for(std::size_t i=0;i<10;++i) {
      require(selected[i].timestamp_ns==1700000000000000005+static_cast<std::int64_t>(5*i));
      require(selected[i].corners.size()==16);
    }
    fails([&]{sampler.consider({});});
    fails([]{calib::InitialViewSampler invalid(0,10);});
    fails([]{calib::InitialViewSampler invalid(50,201);});
    calib::InitialViewSampler blank(20,10);
    for(unsigned i=0;i<20;++i) blank.consider({});
    require(blank.selected().empty());
    std::cout<<"Bounded temporal selection, nanoseconds and invalid budgets passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
