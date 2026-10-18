#include "calib/observations.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
void require(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
void near(double actual,double expected) { require(std::abs(actual-expected)<1e-14,"coordinate mismatch"); }
template<class F> void fails(F operation) {
  bool failed=false;
  try { operation(); } catch(const calib::Error&) { failed=true; }
  require(failed,"invalid geometry accepted");
}
}
int main() {
  using namespace calib;
  try {
    const Target target{"tag36h11",2,3,10,.2,.25,2};
    const AprilGridGeometry geometry(target);
    near(geometry.width_m(),.7); near(geometry.height_m(),.45);
    const std::array<std::array<double,3>,4> last{{{.5,.25,0},{.7,.25,0},{.7,.45,0},{.5,.45,0}}};
    for(unsigned k=0;k<4;++k) {
      const auto& corner=geometry.corners(15)[k];
      require(corner.point_id==20+k && corner.corner_index==k && corner.tag_id==15,"stable corner identity");
      for(unsigned axis=0;axis<3;++axis) near(corner.position_m[axis],last[k][axis]);
    }
    require(geometry.corners(10)[0].position_m==std::array<double,3>{0,0,0},"origin");
    near(geometry.corners(11)[0].position_m[0],.25);
    near(geometry.corners(13)[0].position_m[1],.25);
    fails([&]{geometry.corners(9);}); fails([&]{geometry.corners(16);});
    auto invalid=target; invalid.spacing_ratio=-.1; fails([&]{AprilGridGeometry grid(invalid);});
    invalid=target; invalid.size_m=std::numeric_limits<double>::infinity(); fails([&]{AprilGridGeometry grid(invalid);});
    invalid=target; invalid.size_m=1e308; fails([&]{AprilGridGeometry grid(invalid);});
    invalid=target; invalid.first_id=std::numeric_limits<int>::max(); fails([&]{AprilGridGeometry grid(invalid);});
    invalid=target; invalid.spacing_ratio=1e20; fails([&]{AprilGridGeometry grid(invalid);});
    auto native=target; native.border_bits=1;
    const AprilGridGeometry native_geometry(native);
    require(native_geometry.corners(15)[2].position_m==geometry.corners(15)[2].position_m,"border must not change physical tag size");

    // Independent perspective fixture: homography with positive X, negative image Y.
    // Four rotations of a non-square projected quad must preserve correspondence identities.
    const FrameRef frame{"cam0",1700000000000000001,"",{1000,800,1,16}};
    Tag tag{15,0,50,{{{100,300},{280,280},{250,100},{120,130}}}};
    for(unsigned rotation=0;rotation<4;++rotation) {
      const auto result=make_observations(geometry,frame,std::span(&tag,1));
      require(result.corners.size()==4 && result.rejected.empty(),"valid projected tag rejected");
      for(unsigned k=0;k<4;++k) {
        require(result.corners[k].point_id==20+k,"rotation changed point identity");
        require(result.corners[k].pixel==tag.corners[k],"observation altered original pixel");
      }
      for(auto& p:tag.corners) { const double x=p[0]; p[0]=800-p[1]; p[1]=x; }
    }
    auto reject=[&](Tag bad,const std::string& reason) {
      const auto result=make_observations(geometry,frame,std::span(&bad,1));
      require(result.corners.empty() && result.rejected.size()==1,"invalid tag was partially accepted");
      require(result.rejected[0].reason==reason,"incorrect rejection reason");
    };
    auto bad=tag; bad.id=100; reject(bad,"outside_grid_id_range");
    bad=tag; bad.corners[0][0]=1000; reject(bad,"corner_outside_image");
    bad=tag; bad.corners[0][1]=-.1; reject(bad,"corner_outside_image");
    bad=tag; bad.corners[0][0]=std::numeric_limits<double>::quiet_NaN(); reject(bad,"nonfinite_corner");
    bad=tag; std::swap(bad.corners[1],bad.corners[3]); reject(bad,"invalid_corner_winding_or_degenerate");
    bad=tag; bad.corners[2]=bad.corners[1]; reject(bad,"invalid_corner_winding_or_degenerate");
    const std::array<Tag,2> duplicates{tag,tag};
    const auto duplicate_result=make_observations(geometry,frame,duplicates);
    require(duplicate_result.corners.empty() && duplicate_result.rejected.size()==2,"duplicate ID accepted");
    const auto empty=make_observations(geometry,frame,{});
    require(observations_json(empty)["status"]=="no_correspondences","empty frame lost");
    require(observations_json(empty)["timestamp_ns"].get<std::int64_t>()==frame.timestamp_ns,"timestamp precision lost");
    const auto json=geometry_json(geometry);
    require(json["points"].size()==24 && json["units"]=="m","incomplete geometry export");
    std::cout<<"Grid metric geometry, projection ordering and invalid observations passed\n";
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
