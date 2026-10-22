#include "calib/vi.hpp"
#include "calib/vi_residual.hpp"
#include <ceres/ceres.h>
#include <ceres/product_manifold.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SparseCholesky>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <set>

namespace calib {
namespace {
using Knot=std::array<double,7>;
using BiasKnot=std::array<double,6>;
using PoseManifold=ceres::ProductManifold<ceres::QuaternionManifold,ceres::EuclideanManifold<3>>;
using Vector=Eigen::Vector3d;
using Matrix=Eigen::Matrix3d;
struct Seed { double time; Eigen::Quaterniond rotation; Vector position; };
struct View { const FrameObservations* frame; double time; bool held_out; };
struct Uncertainty { Json covariance; Json observability; };
enum class ResidualKind { visual,imu,bias_prior };
struct ResidualGroup { ceres::ResidualBlockId id; int rows; ResidualKind kind; };
class BootstrapLoss final : public ceres::LossFunction {
 public:
  BootstrapLoss(std::unique_ptr<ceres::LossFunction> base,double time_value,bool is_inertial_value)
      :base_(std::move(base)),sample_time(time_value),inertial(is_inertial_value) {}
  void Evaluate(double squared_norm,double output[3]) const override {
    base_->Evaluate(squared_norm,output);
    for(unsigned index=0;index<3;++index) output[index]*=weight;
  }
 private:
  std::unique_ptr<ceres::LossFunction> base_;
 public:
  double weight{1},sample_time;
  bool inertial;
};
struct SeedAnchor {
  Knot seed;
  template<class T> bool operator()(const T* value,T* residual) const {
    const T inverse[4]={T(seed[0]),T(-seed[1]),T(-seed[2]),T(-seed[3])};
    T difference[4],tangent[3];
    ceres::QuaternionProduct(inverse,value,difference);
    ceres::QuaternionToAngleAxis(difference,tangent);
    for(unsigned axis=0;axis<3;++axis) {
      residual[axis]=tangent[axis]/T(.1);
      residual[axis+3]=(value[axis+4]-T(seed[axis+4]))/T(.05);
    }
    return true;
  }
};
void check_cancelled(const ViOptions& options) {
  if(options.cancelled && options.cancelled()) throw Error(130,"Camera-IMU calibration interrupted");
}
Matrix skew(const Vector& value) {
  Matrix result;
  result<<0,-value.z(),value.y(),value.z(),0,-value.x(),-value.y(),value.x(),0;
  return result;
}
Eigen::Quaterniond quaternion(const double* value) { return {value[0],value[1],value[2],value[3]}; }
Vector vector(const std::array<double,3>& value) { return {value[0],value[1],value[2]}; }
std::array<double,3> array(const Vector& value) { return {value.x(),value.y(),value.z()}; }
vi_detail::SplineState<double> evaluate_at(const std::vector<Knot>& knots,double time,double interval) {
  const int segment=static_cast<int>(std::floor(time/interval));
  if(segment<0 || segment+3>=static_cast<int>(knots.size())) throw Error(3,"Spline query outside supported interval");
  const double* blocks[4];
  for(unsigned index=0;index<4;++index) blocks[index]=knots[static_cast<std::size_t>(segment)+index].data();
  return vi_detail::evaluate(blocks,time/interval-segment,interval);
}
struct BiasQuery { std::size_t index{}; double fraction{}; };
BiasQuery bias_query(double time,double start,double interval,std::size_t count) {
  if(count==1) return {};
  const double coordinate=(time-start)/interval;
  if(coordinate<-1e-9 || coordinate>static_cast<double>(count-1)+1e-9)
    throw Error(3,"Bias query outside supported interval");
  const double bounded=std::clamp(coordinate,0.0,std::nextafter(static_cast<double>(count-1),0.0));
  const auto index=std::min(static_cast<std::size_t>(std::floor(bounded)),count-2);
  return {index,bounded-static_cast<double>(index)};
}
BiasKnot evaluate_bias(const std::vector<BiasKnot>& biases,const BiasQuery& query) {
  if(biases.size()==1) return biases.front();
  BiasKnot result;
  for(unsigned component=0;component<result.size();++component)
    result[component]=(1-query.fraction)*biases[query.index][component]+query.fraction*biases[query.index+1][component];
  return result;
}
struct Cancellation final : ceres::IterationCallback {
  const ViOptions& options;
  explicit Cancellation(const ViOptions& input):options(input) {}
  ceres::CallbackReturnType operator()(const ceres::IterationSummary& iteration) override {
    if(iteration.iteration%10==0) std::cerr<<"VI iteration "<<iteration.iteration<<": cost "<<iteration.cost<<'\n';
    return options.cancelled && options.cancelled()?ceres::SOLVER_ABORT:ceres::SOLVER_CONTINUE;
  }
};
ceres::Solver::Summary solve(ceres::Problem& problem,const ViOptions& options,unsigned iterations) {
  Cancellation callback(options);
  ceres::Solver::Options settings;
  settings.max_num_iterations=static_cast<int>(iterations);
  settings.num_threads=static_cast<int>(options.threads);
  settings.linear_solver_type=ceres::SPARSE_NORMAL_CHOLESKY;
  settings.function_tolerance=1e-7;
  settings.gradient_tolerance=1e-9;
  settings.parameter_tolerance=1e-9;
  settings.logging_type=ceres::SILENT;
  settings.callbacks.push_back(&callback);
  ceres::Solver::Summary summary;
  ceres::Solve(settings,&problem,&summary);
  check_cancelled(options);
  if(!summary.IsSolutionUsable()) throw Error(3,"VI optimizer failed: "+summary.BriefReport());
  return summary;
}
double quantile(std::vector<double> values,double fraction) {
  if(values.empty()) throw Error(3,"Missing residual samples");
  std::sort(values.begin(),values.end());
  return values[static_cast<std::size_t>(fraction*static_cast<double>(values.size()-1))];
}
Uncertainty estimate_uncertainty(ceres::Problem& problem,std::vector<Knot>& knots,std::vector<BiasKnot>& biases,double* gravity,
    double* extrinsic,double* offset,const std::vector<ResidualGroup>& residual_groups,unsigned threads) {
  const auto begin=std::chrono::steady_clock::now();
  std::vector<double*> ordered;
  std::vector<std::string> column_labels;
  int nuisance_columns=0;
  for(std::size_t index=0;index<knots.size();++index) if(!problem.IsParameterBlockConstant(knots[index].data())) {
    ordered.push_back(knots[index].data()); nuisance_columns+=6;
    for(const char* component:{"rx","ry","rz","tx","ty","tz"})
      column_labels.push_back("trajectory["+std::to_string(index)+"]."+component);
  }
  for(std::size_t index=0;index<biases.size();++index) {
    ordered.push_back(biases[index].data()); nuisance_columns+=6;
    for(const char* component:{"gyro_x","gyro_y","gyro_z","accel_x","accel_y","accel_z"})
      column_labels.push_back("bias["+std::to_string(index)+"]."+component);
  }
  ordered.push_back(gravity); nuisance_columns+=2;
  column_labels.push_back("gravity.tangent_0"); column_labels.push_back("gravity.tangent_1");
  ordered.push_back(extrinsic);
  ordered.push_back(offset);
  ceres::Problem::EvaluateOptions evaluation;
  evaluation.parameter_blocks=std::move(ordered);
  for(const auto& group:residual_groups) evaluation.residual_blocks.push_back(group.id);
  evaluation.apply_loss_function=true;
  evaluation.num_threads=static_cast<int>(threads);
  ceres::CRSMatrix jacobian;
  std::vector<double> residuals;
  if(!problem.Evaluate(evaluation,nullptr,&residuals,nullptr,&jacobian) || jacobian.num_cols!=nuisance_columns+7)
    return {nullptr,{{"status","jacobian_evaluation_failed"},{"method","targeted sparse Schur complement"}}};
  using RowSparse=Eigen::SparseMatrix<double,Eigen::RowMajor,int>;
  using ColSparse=Eigen::SparseMatrix<double,Eigen::ColMajor,int>;
  const Eigen::Map<const RowSparse> full(jacobian.num_rows,jacobian.num_cols,static_cast<int>(jacobian.values.size()),
    jacobian.rows.data(),jacobian.cols.data(),jacobian.values.data());
  ColSparse nuisance=full.leftCols(nuisance_columns);
  for(int column=0;column<nuisance_columns;++column) {
    double squared_norm=0;
    for(ColSparse::InnerIterator entry(nuisance,column);entry;++entry) squared_norm+=entry.value()*entry.value();
    if(!std::isfinite(squared_norm) || squared_norm<=0) return {nullptr,{
      {"status","nuisance_zero_column"},{"method","targeted sparse Schur complement"},{"column",column}}};
    const double scale=std::sqrt(squared_norm);
    for(ColSparse::InnerIterator entry(nuisance,column);entry;++entry) entry.valueRef()/=scale;
  }
  const Eigen::Matrix<double,Eigen::Dynamic,7> target=full.rightCols<7>();
  ColSparse nuisance_information=nuisance.transpose()*nuisance;
  nuisance_information.makeCompressed();
  const Eigen::Matrix<double,Eigen::Dynamic,7> cross=nuisance.transpose()*target;
  const Eigen::Matrix<double,7,7> target_information=target.transpose()*target;
  Eigen::SimplicialLDLT<ColSparse> nuisance_factor;
  nuisance_factor.compute(nuisance_information);
  Json nuisance_diagnostics={{"status","full_rank"}};
  constexpr double nuisance_rank_threshold=1e-12;
  const bool nuisance_factor_valid=nuisance_factor.info()==Eigen::Success &&
    nuisance_factor.vectorD().size()==nuisance_columns && nuisance_factor.vectorD().allFinite();
  const double maximum_pivot=nuisance_factor_valid?nuisance_factor.vectorD().cwiseAbs().maxCoeff():0;
  const bool nuisance_full_rank=nuisance_factor_valid && maximum_pivot>0 &&
    nuisance_factor.vectorD().minCoeff()>nuisance_rank_threshold*maximum_pivot;
  if(!nuisance_full_rank) {
    nuisance_diagnostics={{"status","rank_deficient"},{"nuisance_tangent_dimension",nuisance_columns},
      {"relative_pivot_threshold",nuisance_rank_threshold}};
    if(nuisance_factor_valid && maximum_pivot>0) {
      std::size_t nonpositive=0,small=0;
      Json small_pivots=Json::array();
      for(const double value:nuisance_factor.vectorD()) {
        if(value<=0) ++nonpositive;
        if(std::abs(value)<=nuisance_rank_threshold*maximum_pivot) ++small;
      }
      const auto& permutation=nuisance_factor.permutationP().indices();
      for(Eigen::Index index=0;index<nuisance_factor.vectorD().size();++index) {
        if(std::abs(nuisance_factor.vectorD()[index])<=nuisance_rank_threshold*maximum_pivot) {
          const auto column=permutation.size()?static_cast<std::size_t>(permutation[index]):static_cast<std::size_t>(index);
          small_pivots.push_back({{"pivot",nuisance_factor.vectorD()[index]},{"column",column},
            {"parameter",column<column_labels.size()?column_labels[column]:"unknown"}});
        }
      }
      nuisance_diagnostics["minimum_pivot"]=nuisance_factor.vectorD().minCoeff();
      nuisance_diagnostics["maximum_absolute_pivot"]=maximum_pivot;
      nuisance_diagnostics["nonpositive_pivots"]=nonpositive;
      nuisance_diagnostics["pivots_below_relative_1e-12"]=small;
      nuisance_diagnostics["small_pivots"]=std::move(small_pivots);
    }
  }
  Eigen::Matrix<double,7,7> schur;
  Eigen::Matrix<double,Eigen::Dynamic,7> eliminated;
  std::string observability_status="full_rank_local_linearization";
  if(nuisance_full_rank) {
    eliminated=nuisance_factor.solve(cross);
    if(nuisance_factor.info()!=Eigen::Success || !eliminated.allFinite())
      return {nullptr,{{"status","nuisance_solve_failed"},{"method","targeted sparse Schur complement"}}};
    schur=target_information-cross.transpose()*eliminated;
  } else {
    const auto regularized_schur=[&](double damping,Eigen::Matrix<double,7,7>& result) {
      ColSparse regularized=nuisance_information;
      for(int diagonal=0;diagonal<nuisance_columns;++diagonal) regularized.coeffRef(diagonal,diagonal)+=damping;
      regularized.makeCompressed();
      Eigen::SimplicialLDLT<ColSparse> factor;
      factor.compute(regularized);
      if(factor.info()!=Eigen::Success) return false;
      const Eigen::Matrix<double,Eigen::Dynamic,7> regularized_eliminated=factor.solve(cross);
      if(factor.info()!=Eigen::Success || !regularized_eliminated.allFinite()) return false;
      result=target_information-cross.transpose()*regularized_eliminated;
      result=(result+result.transpose())/2.0;
      return result.allFinite() && result.diagonal().minCoeff()>0;
    };
    constexpr double coarse_damping=1e-8,fine_damping=1e-10,stability_limit=1e-3;
    Eigen::Matrix<double,7,7> coarse_schur,fine_schur;
    if(!regularized_schur(coarse_damping,coarse_schur) || !regularized_schur(fine_damping,fine_schur)) {
      nuisance_diagnostics["status"]="rank_deficient_and_target_coupled";
      return {nullptr,{{"status","nuisance_rank_deficient"},{"method","targeted sparse Schur complement"},
        {"nuisance",std::move(nuisance_diagnostics)}}};
    }
    const double relative_change=(fine_schur-coarse_schur).norm()/std::max(fine_schur.norm(),1e-30);
    nuisance_diagnostics["pseudoinverse_limit"]={{"coarse_damping",coarse_damping},{"fine_damping",fine_damping},
      {"schur_relative_change",relative_change},{"stability_limit",stability_limit}};
    if(relative_change>stability_limit) {
      nuisance_diagnostics["status"]="rank_deficient_and_target_coupled";
      return {nullptr,{{"status","nuisance_nullspace_couples_target"},{"method","targeted sparse Schur complement"},
        {"nuisance",std::move(nuisance_diagnostics)}}};
    }
    nuisance_diagnostics["status"]="rank_deficient_target_stable";
    schur=fine_schur;
    observability_status="target_full_rank_with_nuisance_nullspace";
  }
  schur=(schur+schur.transpose())/2.0;
  if(!schur.allFinite() || schur.diagonal().minCoeff()<=0)
    return {nullptr,{{"status","target_information_invalid"},{"method","targeted sparse Schur complement"}}};
  const Eigen::Matrix<double,7,1> information_scale=schur.diagonal().cwiseSqrt();
  Eigen::Matrix<double,7,7> normalized_information;
  for(unsigned row=0;row<7;++row) for(unsigned column=0;column<7;++column)
    normalized_information(row,column)=schur(row,column)/(information_scale[row]*information_scale[column]);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>> information_spectrum(normalized_information);
  constexpr double rank_threshold=1e-12;
  if(information_spectrum.info()!=Eigen::Success || information_spectrum.eigenvalues().minCoeff()<=0 ||
      information_spectrum.eigenvalues().minCoeff()/information_spectrum.eigenvalues().maxCoeff()<rank_threshold)
    return {nullptr,{{"status","target_rank_deficient"},{"method","targeted sparse Schur complement"},
      {"normalized_information_rank_threshold",rank_threshold}}};
  Eigen::LDLT<Eigen::Matrix<double,7,7>> target_factor(schur);
  if(target_factor.info()!=Eigen::Success || !target_factor.isPositive())
    return {nullptr,{{"status","target_factorization_failed"},{"method","targeted sparse Schur complement"}}};
  const Eigen::Matrix<double,7,7> tangent=target_factor.solve(Eigen::Matrix<double,7,7>::Identity());
  Eigen::Matrix<double,7,7> scaling=Eigen::Matrix<double,7,7>::Identity();
  scaling.diagonal().head<3>().setConstant(2.0);
  Eigen::Matrix<double,7,7> physical=scaling*tangent*scaling;
  physical=(physical+physical.transpose())/2.0;
  if(!physical.allFinite() || physical.diagonal().minCoeff()<=0) return {nullptr,{
    {"status","invalid_covariance"},{"method","targeted sparse Schur complement"}}};
  const Eigen::Matrix<double,7,1> standard_deviation=physical.diagonal().cwiseSqrt();
  Eigen::Matrix<double,7,7> correlation;
  for(unsigned row=0;row<7;++row) for(unsigned column=0;column<7;++column)
    correlation(row,column)=physical(row,column)/(standard_deviation[row]*standard_deviation[column]);
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>> spectrum(correlation);
  if(spectrum.info()!=Eigen::Success || spectrum.eigenvalues().minCoeff()<=0) return {nullptr,{
    {"status","invalid_correlation_spectrum"},{"method","targeted sparse Schur complement"}}};
  Json matrix=Json::array();
  for(unsigned row=0;row<7;++row) {
    Json values=Json::array();
    for(unsigned column=0;column<7;++column) values.push_back(physical(row,column));
    matrix.push_back(std::move(values));
  }
  Json correlation_matrix=Json::array();
  for(unsigned row=0;row<7;++row) {
    Json values=Json::array();
    for(unsigned column=0;column<7;++column) values.push_back(correlation(row,column));
    correlation_matrix.push_back(std::move(values));
  }
  Json all_eigenvalues=Json::array();
  for(const double value:spectrum.eigenvalues()) all_eigenvalues.push_back(value);
  Json information_eigenvalues=Json::array();
  for(const double value:information_spectrum.eigenvalues()) information_eigenvalues.push_back(value);
  Json sandwich={{"status","unavailable"},{"qualification","candidate_not_qualified"}};
  int grouped_rows=0;
  for(const auto& group:residual_groups) grouped_rows+=group.rows;
  if(nuisance_full_rank && grouped_rows==jacobian.num_rows && residuals.size()==static_cast<std::size_t>(jacobian.num_rows)) {
    const RowSparse normalized_nuisance=nuisance;
    Eigen::Matrix<double,7,7> meat=Eigen::Matrix<double,7,7>::Zero();
    int first_row=0;
    for(const auto& group:residual_groups) {
      Eigen::Matrix<double,7,1> score=Eigen::Matrix<double,7,1>::Zero();
      if(group.kind!=ResidualKind::bias_prior) {
        for(int row=first_row;row<first_row+group.rows;++row) {
          Eigen::Matrix<double,1,7> reduced=target.row(row);
          for(RowSparse::InnerIterator entry(normalized_nuisance,row);entry;++entry)
            reduced.noalias()-=entry.value()*eliminated.row(entry.col());
          score.noalias()+=reduced.transpose()*residuals[static_cast<std::size_t>(row)];
        }
        meat.noalias()+=score*score.transpose();
      }
      first_row+=group.rows;
    }
    Eigen::Matrix<double,7,7> candidate=scaling*tangent*meat*tangent*scaling;
    candidate=(candidate+candidate.transpose())/2.0;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>> candidate_spectrum(candidate);
    if(candidate.allFinite() && candidate.diagonal().minCoeff()>0 && candidate_spectrum.info()==Eigen::Success &&
        candidate_spectrum.eigenvalues().minCoeff()>0) {
      const Eigen::Matrix<double,7,1> candidate_std=candidate.diagonal().cwiseSqrt();
      Json candidate_matrix=Json::array(),candidate_correlation=Json::array();
      for(unsigned row=0;row<7;++row) {
        Json matrix_row=Json::array(),correlation_row=Json::array();
        for(unsigned column=0;column<7;++column) {
          matrix_row.push_back(candidate(row,column));
          correlation_row.push_back(candidate(row,column)/(candidate_std[row]*candidate_std[column]));
        }
        candidate_matrix.push_back(std::move(matrix_row));
        candidate_correlation.push_back(std::move(correlation_row));
      }
      sandwich={{"status","candidate"},{"qualification","not_qualified_for_default_use"},
        {"matrix",std::move(candidate_matrix)},
        {"standard_deviation",{{"rotation_left_rad",{candidate_std[0],candidate_std[1],candidate_std[2]}},
          {"translation_m",{candidate_std[3],candidate_std[4],candidate_std[5]}},{"timeshift_s",candidate_std[6]}}},
        {"correlation_matrix",std::move(candidate_correlation)},
        {"method","HC0 block-score sandwich around the Ceres robust Gauss-Newton bread after sparse nuisance elimination"},
        {"score_groups",{{"visual_corner_blocks",static_cast<std::size_t>(std::count_if(residual_groups.begin(),residual_groups.end(),
          [](const ResidualGroup& group){ return group.kind==ResidualKind::visual; }))},
          {"imu_sample_blocks",static_cast<std::size_t>(std::count_if(residual_groups.begin(),residual_groups.end(),
          [](const ResidualGroup& group){ return group.kind==ResidualKind::imu; }))},
          {"excluded_bias_prior_blocks",static_cast<std::size_t>(std::count_if(residual_groups.begin(),residual_groups.end(),
          [](const ResidualGroup& group){ return group.kind==ResidualKind::bias_prior; }))}}},
        {"robust_score","Ceres-consistent corrected Jacobian transpose corrected residual"}};
    } else sandwich["reason"]="candidate covariance is not strictly positive definite";
  } else if(!nuisance_full_rank) sandwich["reason"]="nuisance elimination required regularization";
  else sandwich["reason"]="residual group metadata does not match evaluated rows";
  const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  return {{
      {"status","available"},
      {"parameter_order",{"delta_theta_left_x_rad","delta_theta_left_y_rad","delta_theta_left_z_rad",
        "delta_translation_x_m","delta_translation_y_m","delta_translation_z_m","delta_timeshift_s"}},
      {"matrix",std::move(matrix)},
      {"standard_deviation",{{"rotation_left_rad",{standard_deviation[0],standard_deviation[1],standard_deviation[2]}},
        {"translation_m",{standard_deviation[3],standard_deviation[4],standard_deviation[5]}},
        {"timeshift_s",standard_deviation[6]}}},
      {"correlation_matrix",std::move(correlation_matrix)},
      {"sandwich",std::move(sandwich)},
      {"method","targeted sparse normal-equation Schur complement of the full whitened local Jacobian with unit-norm nuisance columns"},
      {"marginalized_parameters","trajectory controls, gyroscope/accelerometer bias controls and gravity direction"},
      {"linearization",{{"residuals",jacobian.num_rows},{"nuisance_tangent_dimension",nuisance_columns},
        {"jacobian_nonzeros",jacobian.values.size()},{"seconds",elapsed}}},
      {"nuisance_elimination",nuisance_diagnostics},
      {"robust_loss","Ceres loss correction applied"},
      {"statistical_interpretation","primary matrix is the local inverse Gauss-Newton Hessian under declared independent noise; sandwich is an unqualified candidate"},
      {"perturbation","R_cam_imu' = Exp(delta_theta_left) R_cam_imu; t_cam_imu' = t_cam_imu + delta_translation; offset' = offset + delta_timeshift"}
    },{
      {"status",observability_status},
      {"method","normalized 7x7 marginalized information after sparse nuisance elimination; AprilGrid world gauge fixed"},
      {"normalized_information_eigenvalues",std::move(information_eigenvalues)},
      {"normalized_information_eigenvalue_ratio",information_spectrum.eigenvalues().minCoeff()/information_spectrum.eigenvalues().maxCoeff()},
      {"normalized_information_rank_threshold",rank_threshold},
      {"correlation_eigenvalues",std::move(all_eigenvalues)},
      {"correlation_eigenvalue_ratio",spectrum.eigenvalues().minCoeff()/spectrum.eigenvalues().maxCoeff()},
      {"scope","local numerical rank and correlation only; not an accuracy certificate"}
    }};
}
Json estimate_bootstrap(ceres::Problem& problem,std::vector<Knot>& knots,std::vector<BiasKnot>& biases,
    std::array<double,3>& gravity,Knot& extrinsic,double& offset,const std::vector<BootstrapLoss*>& losses,
    double duration,double interval,const ViOptions& options) {
  if(options.bootstrap_replicates==0) return nullptr;
  const auto begin=std::chrono::steady_clock::now();
  const std::size_t block_count=static_cast<std::size_t>(std::ceil(duration/options.bootstrap_block_s));
  const auto central_knots=knots;
  const auto central_biases=biases;
  const auto central_gravity=gravity;
  const auto central_extrinsic=extrinsic;
  const double central_offset=offset;
  std::vector<bool> nominal_constant(knots.size());
  for(std::size_t index=0;index<knots.size();++index)
    nominal_constant[index]=problem.IsParameterBlockConstant(knots[index].data());
  std::mt19937_64 generator(options.bootstrap_seed);
  std::uniform_int_distribution<std::size_t> distribution(0,block_count-1);
  std::vector<Eigen::Matrix<double,7,1>> deltas;
  Json trials=Json::array();
  const auto restore=[&] {
    knots=central_knots; biases=central_biases; gravity=central_gravity; extrinsic=central_extrinsic; offset=central_offset;
    for(auto* loss:losses) loss->weight=1;
    for(std::size_t index=0;index<knots.size();++index) {
      if(nominal_constant[index]) problem.SetParameterBlockConstant(knots[index].data());
      else problem.SetParameterBlockVariable(knots[index].data());
    }
  };
  for(unsigned replicate=0;replicate<options.bootstrap_replicates;++replicate) {
    restore();
    check_cancelled(options);
    std::vector<unsigned> multiplicities(block_count);
    for(std::size_t draw=0;draw<block_count;++draw) ++multiplicities[distribution(generator)];
    const auto distinct=static_cast<unsigned>(std::count_if(multiplicities.begin(),multiplicities.end(),
      [](unsigned value){ return value!=0; }));
    Json trial={{"replicate",replicate},{"multiplicities",multiplicities},{"distinct_blocks",distinct}};
    if(distinct<3) {
      trial["status"]="rejected"; trial["reason"]="fewer_than_three_distinct_time_blocks";
      trials.push_back(std::move(trial));
      continue;
    }
    std::vector<bool> active_knots(knots.size());
    std::set<double> visual_times,imu_times;
    std::size_t active_visual_residuals=0;
    for(auto* loss:losses) {
      const double time=loss->sample_time-(loss->inertial?central_offset:0.0);
      const auto block=std::min(static_cast<std::size_t>(std::max(time,0.0)/options.bootstrap_block_s),block_count-1);
      loss->weight=static_cast<double>(multiplicities[block]);
      if(loss->weight==0) continue;
      (loss->inertial?imu_times:visual_times).insert(loss->sample_time);
      if(!loss->inertial) ++active_visual_residuals;
      const int segment=static_cast<int>(std::floor(time/interval));
      if(segment<0 || segment+3>=static_cast<int>(knots.size())) continue;
      for(unsigned control=0;control<3;++control) active_knots[static_cast<std::size_t>(segment)+control]=true;
      if(time/interval-segment>1e-12) active_knots[static_cast<std::size_t>(segment)+3]=true;
    }
    if(visual_times.size()<24 || active_visual_residuals<200 || imu_times.size()<100) {
      trial["status"]="rejected"; trial["reason"]="insufficient_unique_weighted_observations";
      trials.push_back(std::move(trial));
      continue;
    }
    for(std::size_t index=0;index<knots.size();++index) {
      if(nominal_constant[index] || !active_knots[index]) problem.SetParameterBlockConstant(knots[index].data());
      else problem.SetParameterBlockVariable(knots[index].data());
    }
    try {
      const auto summary=solve(problem,options,options.bootstrap_max_iterations);
      if(summary.termination_type!=ceres::CONVERGENCE) throw Error(3,"bootstrap optimizer did not converge");
      if(std::abs(offset)>0.98*options.max_offset_s) throw Error(3,"bootstrap offset reaches search boundary");
      const Eigen::Quaterniond relative=quaternion(extrinsic.data())*quaternion(central_extrinsic.data()).conjugate();
      const Eigen::AngleAxisd angle_axis(relative);
      Eigen::Matrix<double,7,1> delta;
      delta.head<3>().setZero();
      if(std::abs(angle_axis.angle())>=1e-15) delta.head<3>()=angle_axis.axis()*angle_axis.angle();
      for(unsigned axis=0;axis<3;++axis) delta[axis+3]=extrinsic[axis+4]-central_extrinsic[axis+4];
      delta[6]=offset-central_offset;
      if(!delta.allFinite()) throw Error(3,"bootstrap estimate is nonfinite");
      deltas.push_back(delta);
      trial["status"]="ok";
      trial["delta"]={delta[0],delta[1],delta[2],delta[3],delta[4],delta[5],delta[6]};
      trial["iterations"]=summary.iterations.size();
    } catch(const Error& error) {
      if(error.code==130) { restore(); throw; }
      trial["status"]="failed"; trial["reason"]=error.what();
    }
    trials.push_back(std::move(trial));
  }
  restore();
  Json result={{"status","unavailable"},{"qualification","not_qualified_for_default_use"},
    {"method","non-overlapping temporal block bootstrap with replacement and exact robust-loss multiplicities"},
    {"block_duration_s",options.bootstrap_block_s},{"blocks",block_count},{"seed",options.bootstrap_seed},
    {"requested_replicates",options.bootstrap_replicates},{"successful_replicates",deltas.size()},
    {"failed_replicates",options.bootstrap_replicates-deltas.size()},{"trials",std::move(trials)},
    {"parameter_order",{"delta_theta_left_x_rad","delta_theta_left_y_rad","delta_theta_left_z_rad",
      "delta_translation_x_m","delta_translation_y_m","delta_translation_z_m","delta_timeshift_s"}},
    {"scope","conditional temporal resampling within one acquisition; fixed intrinsics and the nominal bias temporal model"}};
  if(deltas.size()>=8) {
    Eigen::Matrix<double,7,1> mean=Eigen::Matrix<double,7,1>::Zero();
    for(const auto& delta:deltas) mean+=delta;
    mean/=static_cast<double>(deltas.size());
    Eigen::Matrix<double,7,7> covariance=Eigen::Matrix<double,7,7>::Zero();
    for(const auto& delta:deltas) covariance.noalias()+=(delta-mean)*(delta-mean).transpose();
    covariance/=static_cast<double>(deltas.size()-1);
    covariance=(covariance+covariance.transpose())/2.0;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,7,7>> spectrum(covariance);
    if(covariance.allFinite() && covariance.diagonal().minCoeff()>0 && spectrum.info()==Eigen::Success &&
        spectrum.eigenvalues().minCoeff()>0) {
      Json matrix=Json::array();
      for(unsigned row=0;row<7;++row) {
        Json values=Json::array();
        for(unsigned column=0;column<7;++column) values.push_back(covariance(row,column));
        matrix.push_back(std::move(values));
      }
      result["status"]="candidate";
      result["matrix"]=std::move(matrix);
      result["mean_delta"]={mean[0],mean[1],mean[2],mean[3],mean[4],mean[5],mean[6]};
      result["standard_deviation"]={{"rotation_left_rad",{std::sqrt(covariance(0,0)),std::sqrt(covariance(1,1)),std::sqrt(covariance(2,2))}},
        {"translation_m",{std::sqrt(covariance(3,3)),std::sqrt(covariance(4,4)),std::sqrt(covariance(5,5))}},
        {"timeshift_s",std::sqrt(covariance(6,6))}};
    } else result["reason"]="empirical covariance is not strictly positive definite";
  } else result["reason"]="fewer than eight successful replicates";
  result["seconds"]=std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
  return result;
}
}

ViCalibration calibrate_vi(const AprilGridGeometry& geometry,const CameraCalibration& camera,
    std::span<const FrameObservations> frames,std::span<const ImuSample> imu,const ViOptions& options) {
  check_cancelled(options);
  for(double value:{options.noise.gyroscope_noise_density,options.noise.accelerometer_noise_density,
      options.knot_interval_s,options.bias_knot_interval_s,options.max_offset_s,options.pixel_sigma,options.gravity_m_s2})
    if(!std::isfinite(value) || value<=0) throw Error(2,"VI noise, interval, offset bound, pixel sigma and gravity must be positive and finite");
  for(double value:{options.noise.gyroscope_random_walk,options.noise.accelerometer_random_walk})
    if(!std::isfinite(value) || value<0) throw Error(2,"VI bias random walks must be finite and nonnegative");
  const bool variable_bias=options.noise.gyroscope_random_walk>0;
  if(variable_bias!=(options.noise.accelerometer_random_walk>0))
    throw Error(2,"VI requires both positive gyro/accel random walks, or neither for constant biases");
  if(options.threads<1 || options.threads>256 || options.max_iterations<1 || options.max_iterations>1000 ||
      options.knot_interval_s<0.02 || options.knot_interval_s>1 || options.bias_knot_interval_s<0.02 ||
      options.bias_knot_interval_s>10 || options.max_offset_s>0.5)
    throw Error(2,"Invalid VI solver budget or temporal bounds");
  if(options.bootstrap_replicates && (options.bootstrap_replicates<8 || options.bootstrap_replicates>128 ||
      options.bootstrap_max_iterations<1 || options.bootstrap_max_iterations>200 ||
      !std::isfinite(options.bootstrap_block_s) ||
      options.bootstrap_block_s<4*options.knot_interval_s+2*options.max_offset_s))
    throw Error(2,"VI bootstrap requires 8..128 replicates and a block duration covering four knots plus both offset margins");
  if(!options.bootstrap_replicates && options.bootstrap_block_s!=0)
    throw Error(2,"VI bootstrap block duration requires explicit replicates");
  if(camera.line_delay_s!=0) throw Error(8,"Camera-IMU solver supports global shutter only; nonzero line_delay is rejected");
  const auto optical=make_camera_model(camera);
  if(frames.size()<30 || imu.size()<100) throw Error(3,"VI calibration needs at least 30 camera frames and 100 IMU samples");
  std::map<std::uint32_t,std::array<double,3>> points;
  for(const auto& tag:geometry.tags()) for(const auto& corner:tag) points.emplace(corner.point_id,corner.position_m);
  std::int64_t previous=-1;
  for(const auto& frame:frames) {
    if(frame.timestamp_ns<=previous || frame.timestamp_ns<0) throw Error(2,"VI camera timestamps must increase strictly");
    previous=frame.timestamp_ns;
    if(frame.camera!=camera.id || frame.resolution!=std::array<std::uint32_t,2>{camera.width,camera.height}) throw Error(2,"VI frame camera/resolution mismatch");
    std::set<std::uint32_t> seen;
    for(const auto& corner:frame.corners) {
      if(!points.contains(corner.point_id) || !seen.insert(corner.point_id).second ||
          !std::isfinite(corner.pixel[0]) || !std::isfinite(corner.pixel[1])) throw Error(2,"Invalid VI corner observation");
    }
  }
  previous=-1;
  for(const auto& sample:imu) {
    if(sample.timestamp_ns<=previous || sample.timestamp_ns<0) throw Error(2,"VI IMU timestamps must increase strictly");
    previous=sample.timestamp_ns;
    for(double value:sample.values) if(!std::isfinite(value)) throw Error(2,"Nonfinite VI IMU sample");
  }
  if(imu.back().timestamp_ns<=frames.front().timestamp_ns || imu.front().timestamp_ns>=frames.back().timestamp_ns)
    throw Error(3,"No camera-IMU temporal overlap");
  const auto origin=frames.front().timestamp_ns;
  std::vector<Seed> seeds;
  std::vector<View> views;
  Json rejected=Json::array();
  std::size_t eligible=0;
  for(const auto& frame:frames) {
    check_cancelled(options);
    if(frame.corners.size()<12) {
      rejected.push_back({{"timestamp_ns",frame.timestamp_ns},{"reason","fewer_than_12_corners"}});
      continue;
    }
    const double time=relative_seconds(frame.timestamp_ns,origin);
    const bool held_out=++eligible%5==0;
    if(held_out) { views.push_back({&frame,time,true}); continue; }
    try {
      const auto pose=estimate_target_pose(geometry,frame,camera);
      if(pose.rms_px>5) throw Error(3,"initial_pose_rms_above_5_px");
      Matrix rotation;
      Vector translation;
      for(unsigned row=0;row<3;++row) {
        translation[row]=pose.T_cam_target[row][3];
        for(unsigned col=0;col<3;++col) rotation(row,col)=pose.T_cam_target[row][col];
      }
      seeds.push_back({time,Eigen::Quaterniond(rotation.transpose()),-rotation.transpose()*translation});
      views.push_back({&frame,time,false});
    } catch(const Error& error) {
      if(error.code!=3) throw;
      rejected.push_back({{"timestamp_ns",frame.timestamp_ns},{"reason",error.what()}});
    }
  }
  if(seeds.size()<24 || seeds.back().time-seeds.front().time<3) throw Error(3,"Insufficient visual poses/span for VI trajectory");
  std::size_t best_begin=0,best_end=0,window_begin=0;
  for(std::size_t index=1;index<=seeds.size();++index) {
    if(index!=seeds.size() && seeds[index].time-seeds[index-1].time<=1.0) continue;
    if(index-window_begin>=24 && seeds[index-1].time-seeds[window_begin].time>seeds[best_end].time-seeds[best_begin].time) {
      best_begin=window_begin;best_end=index-1;
    }
    window_begin=index;
  }
  if(best_end-best_begin<23 || seeds[best_end].time-seeds[best_begin].time<3)
    throw Error(3,"No continuous visual window with at least 24 training poses over three seconds");
  seeds=std::vector<Seed>(seeds.begin()+static_cast<std::ptrdiff_t>(best_begin),seeds.begin()+static_cast<std::ptrdiff_t>(best_end+1));
  const double start=seeds.front().time,end=seeds.back().time,interval=options.knot_interval_s;
  if(options.bootstrap_replicates && std::ceil((end-start)/options.bootstrap_block_s)<5)
    throw Error(3,"VI bootstrap requires at least five temporal blocks in the selected visual window");
  for(const auto& view:views) if(view.time<start || view.time>=end)
    rejected.push_back({{"timestamp_ns",view.frame->timestamp_ns},{"reason","outside_longest_continuous_visual_window"},{"held_out",view.held_out}});
  const auto count=static_cast<std::size_t>(std::ceil((end-start)/interval))+3;
  if(count>10000) throw Error(2,"VI trajectory exceeds 10000 controls; select a shorter sequence");
  std::vector<Knot> knots(count);
  for(std::size_t index=0;index<count;++index) {
    const double time=start+(static_cast<double>(index)-1)*interval;
    auto upper=std::upper_bound(seeds.begin(),seeds.end(),time,[](double value,const Seed& seed){return value<seed.time;});
    Seed value=seeds.front();
    if(upper==seeds.end()) value=seeds.back();
    else if(upper!=seeds.begin()) {
      const auto& lower=*std::prev(upper);
      const double fraction=(time-lower.time)/(upper->time-lower.time);
      value.rotation=lower.rotation.slerp(fraction,upper->rotation);
      value.position=(1-fraction)*lower.position+fraction*upper->position;
    }
    knots[index]={value.rotation.w(),value.rotation.x(),value.rotation.y(),value.rotation.z(),value.position.x(),value.position.y(),value.position.z()};
  }
  ceres::Problem problem;
  for(auto& knot:knots) problem.AddParameterBlock(knot.data(),7,new PoseManifold);
  std::vector<ResidualGroup> residual_groups;
  std::vector<BootstrapLoss*> bootstrap_losses;
  const auto robust_loss=[&](double threshold,double sample_time,bool inertial) -> ceres::LossFunction* {
    if(!options.bootstrap_replicates) return new ceres::HuberLoss(threshold);
    auto* loss=new BootstrapLoss(std::make_unique<ceres::HuberLoss>(threshold),sample_time,inertial);
    bootstrap_losses.push_back(loss);
    return loss;
  };
  std::size_t training_corners=0,held_out_corners=0;
  for(const auto& view:views) {
    if(view.time<start || view.time>=end) continue;
    if(view.held_out) { held_out_corners+=view.frame->corners.size(); continue; }
    const double coordinate=(view.time-start)/interval;
    const auto segment=static_cast<std::size_t>(std::floor(coordinate));
    for(const auto& corner:view.frame->corners) {
      auto* cost=new ceres::DynamicAutoDiffCostFunction<vi_detail::VisualResidual,4>(new vi_detail::VisualResidual{
        points.at(corner.point_id),corner.pixel,optical.parameters(),optical.distortion(),coordinate-static_cast<double>(segment),interval,options.pixel_sigma});
      std::vector<double*> blocks;
      for(unsigned index=0;index<4;++index) { cost->AddParameterBlock(7); blocks.push_back(knots[segment+index].data()); }
      cost->SetNumResiduals(2);
      residual_groups.push_back({problem.AddResidualBlock(cost,robust_loss(2/options.pixel_sigma,view.time-start,false),blocks),2,ResidualKind::visual});
      ++training_corners;
    }
  }
  if(training_corners<200 || held_out_corners<48) throw Error(3,"VI requires training and held-out visual corners");
  std::vector<ceres::ResidualBlockId> anchors;
  for(auto& knot:knots) anchors.push_back(problem.AddResidualBlock(
    new ceres::AutoDiffCostFunction<SeedAnchor,6,7>(new SeedAnchor{knot}),nullptr,knot.data()));
  std::cerr<<"Fitting continuous visual trajectory ("<<count<<" controls)\n";
  solve(problem,options,30);
  for(const auto anchor:anchors) problem.RemoveResidualBlock(anchor);
  const double margin=2*interval+options.max_offset_s;
  std::vector<const ImuSample*> used_imu;
  std::vector<double> spacings;
  Vector gyro_mean=Vector::Zero();
  for(std::size_t index=0;index<imu.size();++index) {
    const double time=relative_seconds(imu[index].timestamp_ns,origin)-start;
    if(time<margin || time>end-start-margin) continue;
    if(index && relative_seconds(imu[index].timestamp_ns,imu[index-1].timestamp_ns)>0.05) throw Error(3,"IMU gap above 50 ms in calibration interval");
    used_imu.push_back(&imu[index]);
    gyro_mean+=Vector(imu[index].values[0],imu[index].values[1],imu[index].values[2]);
    if(index) spacings.push_back(relative_seconds(imu[index].timestamp_ns,imu[index-1].timestamp_ns));
  }
  if(used_imu.size()<100) throw Error(3,"Insufficient IMU samples inside spline support and offset margins");
  gyro_mean/=static_cast<double>(used_imu.size());
  Matrix covariance=Matrix::Zero();
  for(const auto* sample:used_imu) {
    const Vector centered=Vector(sample->values[0],sample->values[1],sample->values[2])-gyro_mean;
    covariance+=centered*centered.transpose()/static_cast<double>(used_imu.size());
  }
  const Eigen::SelfAdjointEigenSolver<Matrix> excitation(covariance);
  if(excitation.eigenvalues().minCoeff()<0.005) throw Error(3,"Insufficient multi-axis angular excitation for camera-IMU calibration");
  Knot extrinsic{};
  BiasKnot initial_bias{};
  double offset=0,best_cost=std::numeric_limits<double>::infinity();
  for(int trial=-19;trial<=19;++trial) {
    check_cancelled(options);
    const double candidate=options.max_offset_s*trial/20.;
    Matrix correlation=Matrix::Zero();
    Vector visual_mean=Vector::Zero();
    std::vector<Vector> angular;
    for(const auto* sample:used_imu) {
      const auto state=evaluate_at(knots,relative_seconds(sample->timestamp_ns,origin)-start-candidate,interval);
      angular.push_back(vector(state.omega)); visual_mean+=angular.back();
    }
    visual_mean/=static_cast<double>(angular.size());
    for(std::size_t index=0;index<angular.size();++index) {
      const auto& values=used_imu[index]->values;
      correlation+=(angular[index]-visual_mean)*(Vector(values[0],values[1],values[2])-gyro_mean).transpose();
    }
    const Eigen::JacobiSVD<Matrix> svd(correlation,Eigen::ComputeFullU|Eigen::ComputeFullV);
    Matrix sign=Matrix::Identity(); sign(2,2)=(svd.matrixU()*svd.matrixV().transpose()).determinant();
    const Matrix rotation=svd.matrixU()*sign*svd.matrixV().transpose();
    const Vector candidate_bias=gyro_mean-rotation.transpose()*visual_mean;
    double cost=0;
    for(std::size_t index=0;index<angular.size();++index) {
      const auto& values=used_imu[index]->values;
      cost+=(rotation.transpose()*angular[index]+candidate_bias-Vector(values[0],values[1],values[2])).squaredNorm();
    }
    if(cost<best_cost) {
      best_cost=cost; offset=candidate;
      const Eigen::Quaterniond rotation_q(rotation);
      extrinsic={rotation_q.w(),rotation_q.x(),rotation_q.y(),rotation_q.z(),0,0,0};
      for(unsigned axis=0;axis<3;++axis) initial_bias[axis]=candidate_bias[axis];
    }
  }
  if(std::abs(offset)>=0.999*options.max_offset_s)
    throw Error(3,"Initial IMU offset reaches search boundary; increase the explicit bound or improve motion excitation");
  Eigen::Matrix<double,9,9> normal=Eigen::Matrix<double,9,9>::Zero();
  Eigen::Matrix<double,9,1> rhs=Eigen::Matrix<double,9,1>::Zero();
  const Matrix rotation_ci=quaternion(extrinsic.data()).toRotationMatrix();
  for(const auto* sample:used_imu) {
    const auto state=evaluate_at(knots,relative_seconds(sample->timestamp_ns,origin)-start-offset,interval);
    const Matrix rotation_wc=quaternion(state.rotation.data()).toRotationMatrix();
    Eigen::Matrix<double,3,9> design;
    design.leftCols<3>()=rotation_ci.transpose()*(skew(vector(state.alpha))+skew(vector(state.omega))*skew(vector(state.omega)));
    design.middleCols<3>(3)=-rotation_ci.transpose()*rotation_wc.transpose();
    design.rightCols<3>()=Matrix::Identity();
    const Vector measured(sample->values[3],sample->values[4],sample->values[5]);
    const Vector value=measured-rotation_ci.transpose()*rotation_wc.transpose()*vector(state.acceleration);
    normal+=design.transpose()*design; rhs+=design.transpose()*value;
  }
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double,9,9>> inertial_rank(normal);
  if(inertial_rank.eigenvalues().minCoeff()/inertial_rank.eigenvalues().maxCoeff()<1e-8) throw Error(3,"Degenerate acceleration/lever-arm/gravity initialization");
  const Eigen::Matrix<double,9,1> linear=normal.ldlt().solve(rhs);
  if(!linear.allFinite() || linear.segment<3>(3).norm()<1) throw Error(3,"Invalid gravity/lever-arm initialization");
  const Vector initial_gravity=linear.segment<3>(3).normalized()*options.gravity_m_s2;
  std::cerr<<"VI initialization: offset "<<offset<<" s, gyro RMS "<<std::sqrt(best_cost/static_cast<double>(used_imu.size()))
    <<" rad/s, lever arm "<<linear.head<3>().transpose()<<" m\n";
  std::array<double,3> gravity=array(initial_gravity);
  for(unsigned axis=0;axis<3;++axis) { extrinsic[axis+4]=linear[axis]; initial_bias[axis+3]=linear[axis+6]; }
  const double bias_start=relative_seconds(used_imu.front()->timestamp_ns,origin)-start;
  const double bias_end=relative_seconds(used_imu.back()->timestamp_ns,origin)-start;
  const std::size_t bias_count=variable_bias?
    static_cast<std::size_t>(std::floor((bias_end-bias_start)/options.bias_knot_interval_s))+2:1;
  if(bias_count>10000) throw Error(2,"VI bias trajectory exceeds 10000 controls; increase --vi-bias-knot-interval");
  std::vector<BiasKnot> biases(bias_count,initial_bias);
  problem.AddParameterBlock(extrinsic.data(),7,new PoseManifold);
  for(auto& bias:biases) problem.AddParameterBlock(bias.data(),6);
  problem.AddParameterBlock(gravity.data(),3,new ceres::SphereManifold<3>);
  problem.AddParameterBlock(&offset,1);
  problem.SetParameterLowerBound(&offset,0,-options.max_offset_s);
  problem.SetParameterUpperBound(&offset,0,options.max_offset_s);
  const double spacing=quantile(spacings,0.5);
  const double gyro_sigma=options.noise.gyroscope_noise_density/std::sqrt(spacing);
  const double accel_sigma=options.noise.accelerometer_noise_density/std::sqrt(spacing);
  if(variable_bias) {
    const double gyro_walk_sigma=options.noise.gyroscope_random_walk*std::sqrt(options.bias_knot_interval_s);
    const double accel_walk_sigma=options.noise.accelerometer_random_walk*std::sqrt(options.bias_knot_interval_s);
    for(std::size_t index=1;index<biases.size();++index) {
      auto* cost=new ceres::AutoDiffCostFunction<vi_detail::BiasRandomWalkResidual,6,6,6>(
        new vi_detail::BiasRandomWalkResidual{gyro_walk_sigma,accel_walk_sigma});
      residual_groups.push_back({problem.AddResidualBlock(cost,nullptr,biases[index-1].data(),biases[index].data()),
        6,ResidualKind::bias_prior});
    }
  }
  for(const auto* sample:used_imu) {
    const double time=relative_seconds(sample->timestamp_ns,origin)-start;
    const auto bias_location=bias_query(time,bias_start,options.bias_knot_interval_s,biases.size());
    const int first_segment=static_cast<int>(std::floor((time-options.max_offset_s)/interval));
    const int last_segment=static_cast<int>(std::floor((time+options.max_offset_s)/interval));
    auto* cost=new ceres::DynamicAutoDiffCostFunction<vi_detail::ImuResidual,4>(new vi_detail::ImuResidual{
      sample->values,time,interval,gyro_sigma,accel_sigma,first_segment,last_segment-first_segment+1,
      bias_location.fraction,variable_bias});
    std::vector<double*> blocks{extrinsic.data(),biases[bias_location.index].data(),gravity.data(),&offset};
    for(int size:{7,6,3,1}) cost->AddParameterBlock(size);
    if(variable_bias) { cost->AddParameterBlock(6); blocks.push_back(biases[bias_location.index+1].data()); }
    for(int index=first_segment;index<=last_segment+3;++index) { cost->AddParameterBlock(7); blocks.push_back(knots[static_cast<std::size_t>(index)].data()); }
    cost->SetNumResiduals(6);
    residual_groups.push_back({problem.AddResidualBlock(cost,robust_loss(3,time,true),blocks),6,ResidualKind::imu});
  }
  std::cerr<<"Optimizing camera-IMU trajectory, extrinsics, offset, gravity and "
    <<(variable_bias?"piecewise-linear random-walk biases":"constant biases")<<"\n";
  const auto summary=solve(problem,options,options.max_iterations);
  if(summary.termination_type!=ceres::CONVERGENCE) throw Error(3,"VI optimizer did not converge: "+summary.BriefReport());
  if(std::abs(offset)>0.98*options.max_offset_s) throw Error(3,"Estimated IMU offset reaches search boundary; increase explicit offset bound");
  std::vector<bool> locally_observed_knots(knots.size());
  const auto mark_support=[&](double coordinate) {
    const auto segment=static_cast<std::size_t>(std::floor(coordinate));
    for(unsigned index=0;index<3;++index) locally_observed_knots[segment+index]=true;
    if(coordinate-static_cast<double>(segment)>1e-12) locally_observed_knots[segment+3]=true;
  };
  for(const auto& view:views) if(!view.held_out && view.time>=start && view.time<end)
    mark_support((view.time-start)/interval);
  for(const auto* sample:used_imu)
    mark_support((relative_seconds(sample->timestamp_ns,origin)-start-offset)/interval);
  std::size_t fixed_padding_controls=0;
  for(std::size_t index=0;index<knots.size();++index) if(!locally_observed_knots[index]) {
    problem.SetParameterBlockConstant(knots[index].data());
    ++fixed_padding_controls;
  }
  check_cancelled(options);
  std::cerr<<"Computing marginalized camera-IMU covariance\n";
  const auto uncertainty=estimate_uncertainty(problem,knots,biases,gravity.data(),extrinsic.data(),&offset,residual_groups,options.threads);
  std::cerr<<"VI covariance status: "<<uncertainty.observability.at("status")<<'\n';
  const auto bootstrap=estimate_bootstrap(problem,knots,biases,gravity,extrinsic,offset,bootstrap_losses,end-start,interval,options);
  if(options.bootstrap_replicates) std::cerr<<"VI bootstrap status: "<<bootstrap.at("status")<<'\n';
  check_cancelled(options);
  std::vector<double> training_errors,held_out_errors,residual_x,residual_y;
  std::vector<ViMotionSample> motion_samples;
  std::vector<ViReprojectionFrame> reprojection_frames;
  Json downweighted=Json::array();
  for(const auto& view:views) {
    if(view.time<start || view.time>=end) continue;
    const auto state=evaluate_at(knots,view.time-start,interval);
    ViReprojectionFrame report_frame{view.frame->timestamp_ns,view.time-start,view.held_out,{}};
    report_frame.residuals_px.reserve(view.frame->corners.size());
    for(const auto& corner:view.frame->corners) {
      const Vector source=vector(points.at(corner.point_id))-vector(state.position);
      const auto sensor=quaternion(state.rotation.data()).conjugate()*source;
      const auto projected=optical.project(array(sensor));
      if(!projected) throw Error(3,"Final VI visual projection invalid");
      const double error_x=projected->pixel[0]-corner.pixel[0];
      const double error_y=projected->pixel[1]-corner.pixel[1];
      const double error=std::hypot(error_x,error_y);
      (view.held_out?held_out_errors:training_errors).push_back(error);
      residual_x.push_back(error_x); residual_y.push_back(error_y);
      report_frame.residuals_px.push_back({error_x,error_y});
      if(error>2) downweighted.push_back({{"timestamp_ns",view.frame->timestamp_ns},{"point_id",corner.point_id},{"residual_px",error},{"held_out",view.held_out}});
    }
    motion_samples.push_back({view.time-start,state.position,state.rotation});
    reprojection_frames.push_back(std::move(report_frame));
  }
  if(quantile(held_out_errors,0.5)>5) throw Error(3,"VI held-out median reprojection exceeds 5 pixels");
  auto metrics=[](const std::vector<double>& values) {
    double squared=0; for(double value:values) squared+=value*value;
    return Json{{"corners",values.size()},{"rms_px",std::sqrt(squared/static_cast<double>(values.size()))},
      {"median_px",quantile(values,0.5)},{"p95_px",quantile(values,0.95)}};
  };
  auto signed_metrics=[](const std::vector<double>& values) {
    double sum=0,squared=0,maximum=0;
    for(const double value:values) { sum+=value; squared+=value*value; maximum=std::max(maximum,std::abs(value)); }
    return Json{{"samples",values.size()},{"mean_px",sum/static_cast<double>(values.size())},
      {"rms_px",std::sqrt(squared/static_cast<double>(values.size()))},{"max_abs_px",maximum}};
  };
  double gyro_squared=0,accel_squared=0;
  std::vector<double> gyro_errors,accel_errors;
  std::vector<ViImuReportSample> imu_report_samples;
  imu_report_samples.reserve(used_imu.size());
  for(const auto* sample:used_imu) {
    const double time=relative_seconds(sample->timestamp_ns,origin)-start;
    const auto bias_location=bias_query(time,bias_start,options.bias_knot_interval_s,biases.size());
    const auto sample_bias=evaluate_bias(biases,bias_location);
    const int segment=static_cast<int>(std::floor((time-offset)/interval));
    vi_detail::ImuResidual residual{sample->values,time,interval,1,1,segment,1,
      bias_location.fraction,variable_bias};
    std::vector<const double*> blocks{extrinsic.data(),biases[bias_location.index].data(),gravity.data(),&offset};
    if(variable_bias) blocks.push_back(biases[bias_location.index+1].data());
    for(unsigned index=0;index<4;++index) blocks.push_back(knots[static_cast<std::size_t>(segment)+index].data());
    double values[6];
    if(!residual(blocks.data(),values)) throw Error(3,"Final VI inertial prediction is outside spline support");
    ViImuReportSample report_sample;
    report_sample.timestamp_ns=sample->timestamp_ns;
    report_sample.time_s=time;
    double gyro_norm2=0,accel_norm2=0;
    for(unsigned axis=0;axis<3;++axis) {
      report_sample.measured_gyro_rad_s[axis]=sample->values[axis];
      report_sample.predicted_gyro_rad_s[axis]=sample->values[axis]+values[axis];
      report_sample.gyro_residual_rad_s[axis]=values[axis];
      report_sample.measured_accel_m_s2[axis]=sample->values[axis+3];
      report_sample.predicted_accel_m_s2[axis]=sample->values[axis+3]+values[axis+3];
      report_sample.accel_residual_m_s2[axis]=values[axis+3];
      report_sample.gyro_bias_rad_s[axis]=sample_bias[axis];
      report_sample.accel_bias_m_s2[axis]=sample_bias[axis+3];
      gyro_norm2+=values[axis]*values[axis]; accel_norm2+=values[axis+3]*values[axis+3];
    }
    imu_report_samples.push_back(report_sample);
    gyro_squared+=gyro_norm2;accel_squared+=accel_norm2;
    gyro_errors.push_back(std::sqrt(gyro_norm2));accel_errors.push_back(std::sqrt(accel_norm2));
  }
  if(quantile(gyro_errors,.5)>10*std::sqrt(3.)*gyro_sigma || quantile(accel_errors,.5)>10*std::sqrt(3.)*accel_sigma)
    throw Error(3,"VI inertial median residual exceeds ten declared noise standard deviations: gyro "+std::to_string(quantile(gyro_errors,.5))+
      " rad/s (limit "+std::to_string(10*std::sqrt(3.)*gyro_sigma)+"), accel "+std::to_string(quantile(accel_errors,.5))+
      " m/s^2 (limit "+std::to_string(10*std::sqrt(3.)*accel_sigma)+"); check trajectory resolution, noise/model and initialization");
  ViCalibration result;
  result.camera=camera; result.timeshift_cam_imu=offset;
  const Matrix final_rotation=quaternion(extrinsic.data()).toRotationMatrix();
  result.T_cam_imu={}; result.T_cam_imu[3][3]=1;
  for(unsigned row=0;row<3;++row) {
    for(unsigned col=0;col<3;++col) result.T_cam_imu[row][col]=final_rotation(row,col);
    result.T_cam_imu[row][3]=extrinsic[row+4];
    result.gyro_bias[row]=biases.front()[row]; result.accel_bias[row]=biases.front()[row+3];
  }
  result.gravity_world=gravity;
  result.bootstrap=bootstrap;
  result.report={std::move(motion_samples),std::move(reprojection_frames),std::move(imu_report_samples)};
  result.diagnostics={{"status","converged"},{"iterations",summary.iterations.size()},{"initial_cost",summary.initial_cost},{"final_cost",summary.final_cost},
    {"policy_version",1},{"training",metrics(training_errors)},{"held_out",metrics(held_out_errors)},
    {"held_out_rule","every fifth eligible frame; fixed intrinsics; no held-out pose fitted; same recording"},
    {"gyro_rms_rad_s",std::sqrt(gyro_squared/static_cast<double>(used_imu.size()))},
    {"accel_rms_m_s2",std::sqrt(accel_squared/static_cast<double>(used_imu.size()))},
    {"gyro_median_rad_s",quantile(gyro_errors,.5)},{"accel_median_m_s2",quantile(accel_errors,.5)},
    {"gyro_p95_rad_s",quantile(gyro_errors,.95)},{"accel_p95_m_s2",quantile(accel_errors,.95)},
    {"huber_visual_px",2},{"huber_imu_whitened_block_norm",3},
    {"imu_samples",used_imu.size()},{"imu_excluded_outside_support",imu.size()-used_imu.size()},
    {"imu_median_interval_s",spacing},{"gyro_sample_sigma_rad_s",gyro_sigma},{"accel_sample_sigma_m_s2",accel_sigma},
    {"imu_interval_start_ns",used_imu.front()->timestamp_ns},{"imu_interval_end_ns",used_imu.back()->timestamp_ns},
    {"visual_window_selection","longest continuous training-pose span; maximum gap 1 second"},
    {"visual_window_start_relative_s",start},{"visual_window_end_relative_s",end},
    {"fixed_unreferenced_padding_controls",fixed_padding_controls},
    {"gyro_covariance_eigenvalues",array(excitation.eigenvalues())},{"initial_inertial_design_eigenvalue_ratio",inertial_rank.eigenvalues().minCoeff()/inertial_rank.eigenvalues().maxCoeff()},
    {"max_offset_s",options.max_offset_s},{"pixel_sigma",options.pixel_sigma},{"gyro_noise_density",options.noise.gyroscope_noise_density},
    {"accel_noise_density",options.noise.accelerometer_noise_density},{"gyro_random_walk",options.noise.gyroscope_random_walk},
    {"accel_random_walk",options.noise.accelerometer_random_walk},{"imu_sample_sigma_convention","density / sqrt(median sample interval)"},
    {"intrinsics","fixed"},{"bias_model",variable_bias?"piecewise_linear_random_walk":"constant"},
    {"bias_controls",biases.size()},{"bias_knot_interval_s",variable_bias?Json(options.bias_knot_interval_s):Json(nullptr)},
    {"covariance",uncertainty.covariance},{"observability",uncertainty.observability},
    {"report_series",{{"motion_samples",result.report.motion.size()},
      {"reprojection_frames",result.report.reprojection.size()},
      {"imu_samples",result.report.imu.size()},
      {"reprojection_residuals",residual_x.size()},
      {"residual_definition","projected_pixel - observed_pixel"},
      {"residual_x",signed_metrics(residual_x)},{"residual_y",signed_metrics(residual_y)}}},
    {"rejected_frames",rejected},{"large_visual_residuals",downweighted}};
  result.trajectory={{"type","uniform_cubic_cumulative_SO3_R3_camera"},{"world_frame","fixed AprilGrid"},
    {"transform","T_world_cam"},{"quaternion_order","wxyz"},{"origin_timestamp_ns",origin},
    {"start_relative_s",start},{"knot_interval_s",interval},{"control_layout","qw qx qy qz px py pz"},{"controls",knots}};
  result.bias_trajectory={{"type",variable_bias?"piecewise_linear_random_walk":"constant"},
    {"time_frame","IMU timestamps relative to camera origin"},{"origin_timestamp_ns",origin},
    {"start_relative_s",start+bias_start},{"knot_interval_s",variable_bias?Json(options.bias_knot_interval_s):Json(nullptr)},
    {"control_layout","gyro_x gyro_y gyro_z accel_x accel_y accel_z"},{"controls",biases},
    {"gyroscope_random_walk",options.noise.gyroscope_random_walk},
    {"accelerometer_random_walk",options.noise.accelerometer_random_walk},
    {"increment_sigma_convention","sigma_delta = random_walk * sqrt(knot_interval)"}};
  return result;
}
}
