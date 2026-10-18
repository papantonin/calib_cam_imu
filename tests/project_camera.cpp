// Test-only JSON bridge for comparison with an independent optical implementation.
#include "calib/core.hpp"
#include <iostream>

int main() {
  try {
    calib::Json cases;
    std::cin>>cases;
    calib::Json results=calib::Json::array();
    for(const auto& item:cases) {
      const std::string name=item.at("model");
      if(name!="radtan" && name!="equidistant") throw std::invalid_argument("Unknown test model");
      const calib::CameraModel model(name=="radtan"?calib::DistortionModel::radtan:calib::DistortionModel::equidistant,
                                     item.at("parameters").get<calib::CameraModel::Parameters>());
      const auto value=model.project(item.at("point").get<std::array<double,3>>());
      if(!value) throw std::runtime_error("Test point outside projection domain");
      results.push_back({{"pixel",value->pixel},{"point_jacobian",value->point_jacobian},
                         {"parameter_jacobian",value->parameter_jacobian}});
    }
    std::cout<<results.dump()<<'\n';
    return 0;
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
