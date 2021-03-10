#include "mfplan/mfplanner.h"

#include "mfplan/cv_utils.h"

using namespace lemon;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace mfplan {

Floor::Floor(const cv::Mat map, const int id) : id_(id) {
  map.copyTo(map_img_);
  cv::cvtColor(map, map_, cv::COLOR_BGR2GRAY);

  space_ = std::make_shared<ob::SE2StateSpace>();
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(0, map_.rows);
  bounds.setHigh(1, map_.cols);
  space_->setBounds(bounds);

  space_info_ = std::make_shared<ob::SpaceInformation>(space_);

  auto validity_checker = std::make_shared<Map2DValidityChecker>(
      space_info_, map_);
  space_info_->setStateValidityChecker(validity_checker);
}

std::optional<og::PathGeometric> Floor::find_path(
    dim2::Point<double> start_coords, dim2::Point<double> goal_coords) {
  ob::ScopedState<> start_state(space_);
  start_state[0] = start_coords.x;
  start_state[1] = start_coords.y;
  ob::ScopedState<> goal_state(space_);
  goal_state[0] = goal_coords.x;
  goal_state[1] = goal_coords.y;

  auto problem_def = std::make_shared<ob::ProblemDefinition>(space_info_);
  problem_def->setStartAndGoalStates(start_state, goal_state);

  auto planner = std::make_shared<og::RRTstar>(space_info_);
  planner->setProblemDefinition(problem_def);
  planner->setRange(20.0);
  planner->setup();

  space_info_->printSettings(std::cout);
  problem_def->print(std::cout);

  ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);
  std::optional<og::PathGeometric> path;

  std::cout << "solved: " << solved << std::endl;
  if (bool(solved)) {
    ob::PathPtr p = problem_def->getSolutionPath();
    path = *(p->as<og::PathGeometric>());
    path->interpolate();
    viz_point(start_coords, map_img_);
    viz_point(goal_coords, map_img_);
    viz_path(path, map_img_);
    cv::imwrite("test_out.png", map_img_);
  }
  return path;
}

/*
bool unoccupied(int x, int y) {
  return map_.at<uchar>(x, y) > 210;
}

void print_test_map() {
  int max_width = map_.rows - 1;
  int max_height = map_.cols - 1;
  for (int x = 0; x <= max_width; ++x) {
    for (int y = 0; y <= max_height; ++y) {
      std::cout << !unoccupied(x, y);
    }
    std::cout << std::endl;
  }
}
*/

void Floor::viz_coords(dim2::Point<double> coords) {
  cv::Mat map_img;
  map_.copyTo(map_img);
  cv::Scalar color(0, 255, 0);
  cv::Point ctr(coords.x, coords.y);
  cv::circle(map_img, ctr, 5, color, -1);
  cv::imwrite("test_out_" + std::to_string(id_) + ".png", map_img);
}

MFPlanner::MFPlanner(const std::string& graph_file,
    const std::unordered_map<int, std::string>& map_files) :
  floor_id_(g_), coords_(g_), length_(g_), between_floor_(g_) {

  graphReader(g_, graph_file)
    .nodeMap("floorid", floor_id_)
    .nodeMap("coords", coords_)
    .edgeMap("length", length_)
    .edgeMap("between_floor", between_floor_)
    .run();

  for (auto it = map_files.begin(); it != map_files.end(); ++it) {
    // TODO: make sure map file is nonempty
    id_to_floor_[it->first] = Floor(cv::imread(it->second), it->first);
  }
}

} // namespace mfplan
