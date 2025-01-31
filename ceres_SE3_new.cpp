#include <iostream>
#include <fstream>
#include <string>

#include "ceres/ceres.h"
#include "common/read_g2o.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "common/pose_graph_3d_error_term.h"
#include "common/types.h"

// DEFINE_string(input, "sphere.g2o", "The pose graph definition filename in g2o format.");

namespace ceres {
namespace examples {

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem(const VectorOfConstraints& constraints,
                              MapOfPoses* poses, ceres::Problem* problem) {
  CHECK(poses != NULL);
  CHECK(problem != NULL);
  if (constraints.empty()) {
    LOG(INFO) << "No constraints, no problem to optimize.";
    return;
  }

  ceres::LossFunction* loss_function = NULL;
  ceres::LocalParameterization* quaternion_local_parameterization =
      new EigenQuaternionParameterization;

  for (VectorOfConstraints::const_iterator constraints_iter =
           constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter) {
    const Constraint3d& constraint = *constraints_iter;

    MapOfPoses::iterator pose_begin_iter = poses->find(constraint.id_begin);
    CHECK(pose_begin_iter != poses->end())
        << "Pose with ID: " << constraint.id_begin << " not found.";
    MapOfPoses::iterator pose_end_iter = poses->find(constraint.id_end);
    CHECK(pose_end_iter != poses->end())
        << "Pose with ID: " << constraint.id_end << " not found.";

    const Eigen::Matrix<double, 6, 6> sqrt_information =
        constraint.information.llt().matrixL();
    // Ceres will take ownership of the pointer.
    ceres::CostFunction* cost_function =
        PoseGraph3dErrorTerm::Create(constraint.t_be, sqrt_information);

    problem->AddResidualBlock(cost_function, loss_function,
                              pose_begin_iter->second.p.data(),
                              pose_begin_iter->second.q.coeffs().data(),
                              pose_end_iter->second.p.data(),
                              pose_end_iter->second.q.coeffs().data());

    problem->SetParameterization(pose_begin_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
    problem->SetParameterization(pose_end_iter->second.q.coeffs().data(),
                                 quaternion_local_parameterization);
  }

  // The pose graph optimization problem has six DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigates this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  MapOfPoses::iterator pose_start_iter = poses->begin();
  CHECK(pose_start_iter != poses->end()) << "There are no poses.";
  problem->SetParameterBlockConstant(pose_start_iter->second.p.data());
  problem->SetParameterBlockConstant(pose_start_iter->second.q.coeffs().data());
  
}

// Returns true if the solve was successful.
bool SolveOptimizationProblem(ceres::Problem* problem) {
  CHECK(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 200;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.minimizer_progress_to_stdout = true;
  
  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.FullReport() << '\n';

  return summary.IsSolutionUsable();
}

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, const MapOfPoses& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
           const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    const std::map<int, Pose3d, std::less<int>,
                   Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
        value_type& pair = *poses_iter;
    outfile << pair.first << " " << pair.second.p.transpose() << " "
            << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}

bool WriteG2oFile(const std::string& filename, 
		  const MapOfPoses& poses, 
		  const VectorOfConstraints& constraints){
	std::ofstream outfile(filename.c_str());
	if (!outfile) {
	    LOG(ERROR) << "Error opening the file: " << filename;
	    return false;
	}
	
	for (std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
           const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
	    const std::map<int, Pose3d, std::less<int>,
                   Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
		    value_type& pair = *poses_iter;
	      outfile << pair.second.name() << " " << pair.first << " " 
		      << pair.second.p.transpose() << " "
		      << pair.second.q.x() << " " << pair.second.q.y() << " "
		      << pair.second.q.z() << " " << pair.second.q.w() << '\n';
	}	
	
	for (VectorOfConstraints::const_iterator constraints_iter =
		constraints.begin(); constraints_iter != constraints.end();
	    ++constraints_iter){
	      const Constraint3d& constraint = *constraints_iter;
	      Pose3d pose_r = constraint.t_be;			    
	      outfile << constraint.name() << " " << constraint.id_begin
		      << " " << constraint.id_end << " " 
		      << pose_r.p.transpose() << " "
		      << pose_r.q.x() << " " << pose_r.q.y() << " "
		      << pose_r.q.z() << " " << pose_r.q.w() << " ";
	      
	      const Eigen::Matrix<double, 6, 6> sqrt_information =
			    constraint.information.llt().matrixL();
			    
	      for (size_t i = 0 ; i < 6 ; ++i ){
		  for (size_t j = i ; j < 6 ; ++j){
		      outfile << sqrt_information(i,j) << " ";
		  }
	      }
	      outfile << "\n";
	}
	
	//outfile.close();
	return true;
}
	



}  // namespace examples
}  // namespace ceres

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

//   CHECK(FLAGS_input != "") << "Need to specify the filename to read.";

  ceres::examples::MapOfPoses poses;
  ceres::examples::VectorOfConstraints constraints;

  std::string input_file = argv[1];

  CHECK(ceres::examples::ReadG2oFile(input_file, &poses, &constraints))
      << "Error reading the file: " << input_file;

  std::cout << "Number of poses: " << poses.size() << '\n';
  std::cout << "Number of constraints: " << constraints.size() << '\n';

//   CHECK(ceres::examples::OutputPoses("poses_original.txt", poses))
//       << "Error outputting to poses_original.txt";

  ceres::Problem problem;
  ceres::examples::BuildOptimizationProblem(constraints, &poses, &problem);

  CHECK(ceres::examples::SolveOptimizationProblem(&problem))
      << "The solve was not successful, exiting.";

//   CHECK(ceres::examples::OutputPoses("poses_optimized.txt", poses))
//       << "Error outputting to poses_original.txt";
  
  CHECK(ceres::examples::WriteG2oFile("result_ceres_new.g2o", poses, constraints))
      << "Error outputting to result_Ceres.g2o";

  return 0;
}