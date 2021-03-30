#include <cmath>
#include <limits>
#include "mfplan/cv_utils.h"

#include "mfplan/mfplanner.h"

using namespace lemon;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace mfplan {

bool Map2DValidityChecker::isValid(const ob::State* state) const {
  const auto& se2_state = state->as<ob::SE2StateSpace::StateType>();

  const int x = int(se2_state->getX());
  const int y = int(se2_state->getY());

  if (!si_->satisfiesBounds(state)) {
    return false;
  }

  const int val = map_.at<uchar>(y, x);
  return (val > 210);
}

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
  planner->setup();

  //space_info_->printSettings(std::cout);
  //problem_def->print(std::cout);

  // TODO: timeout should be a parameter
  // TODO: this line is segfaulting and i have NO idea why
  std::cout << "here goes nothing" << std::endl;
  planner->ob::Planner::solve(5.0);
  std::cout << "did i die?" << std::endl;
  ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);
  std::optional<og::PathGeometric> path;

  std::cout << "solved: " << solved << std::endl;
  if (bool(solved)) {
    ob::PathPtr p = problem_def->getSolutionPath();
    path = *(p->as<og::PathGeometric>());
    path->interpolate();
    //viz_point(start_coords, map_img_);
    //viz_point(goal_coords, map_img_);
    //viz_path(path, map_img_);
    //cv::imwrite("test_out.png", map_img_);
  }
  return path;
}

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
  floor_id_(g_), coords_(g_), length_(g_), between_floor_(g_), best_path_(g_),
  best_path_length_(g_), last_timeout_(g_) {

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

  for (ListGraph::EdgeIt e(g_); e != INVALID; ++e) {
    // Between-floor edges should have length pre-populated (or zero)
    // TODO: what length does Lemon give between-floor edges if not
    // populated in the lgf file?
    last_timeout_[e] = 0;
    if (!between_floor_[e]) length_[e] = euclidean_dist(e);
    best_path_length_[e] = std::numeric_limits<float>::max();
  }
}

double MFPlanner::euclidean_dist(ListGraph::Edge e) {
  return std::sqrt((coords_[g_.u(e)] - coords_[g_.v(e)]).normSquare());
}

EdgeList MFPlanner::get_solution_path(
    CoordsAndFloor start, CoordsAndFloor goal) {
  // Does phase 1 only.
  // TODO: add time limit parameter

  // Add start and goal as nodes
  ListGraph::Node start_node = g_.addNode();
  ListGraph::Node goal_node = g_.addNode();
  coords_[start_node] = std::get<0>(start);
  coords_[goal_node] = std::get<0>(goal);
  floor_id_[start_node] = std::get<1>(start);
  floor_id_[goal_node] = std::get<1>(goal);
  // TODO: add edges from start and goal to all other nodes in their floor!
  // hacky solution for now just for testing purposes :3
  g_.addEdge(start_node, g_.nodeFromId(1));
  g_.addEdge(goal_node, g_.nodeFromId(3));

  ListGraph::EdgeMap<double> cost(g_);
  ListGraph::EdgeMap<bool> usable(g_);
  for (ListGraph::EdgeIt e(g_); e != INVALID; ++e) {
    usable[e] = true;
    best_path_length_[e] = std::numeric_limits<float>::max();
    cost[e] = length_[e];
  }

  std::tuple<EdgeList, double> dijk = dijkstra(g_, cost, start_node, goal_node);
  EdgeList dijk_shortest_path = std::get<0>(dijk);
  double dijk_min_path_cost = std::get<1>(dijk);
  double l = dijk_min_path_cost;

  bool solved = false;

  // TODO: get t_0 and k_0 somehow -- class init params?  params of
  // get_solution_path?
  double t = 0.5;
  double k = 1.5;

  while (!solved) {
    while (l < k * dijk_min_path_cost) {
      for (ListGraph::EdgeIt e(g_); e != INVALID; ++e) {
        cost[e] = length_[e];
        if (best_path_[e]) cost[e] = best_path_length_[e];
        if (!usable[e]) cost[e] = std::numeric_limits<float>::max();
      }
      dijk = dijkstra(g_, cost, start_node, goal_node);
      dijk_shortest_path = std::get<0>(dijk);
      l = std::get<1>(dijk);
      std::cout << "new l: " << l << std::endl;
      solved = true;

      for (ListGraph::Edge e : dijk_shortest_path) {
        if (!between_floor_[e] && !best_path_[e]) {
          int floor_id = floor_id_[g_.u(e)];
          std::cout << "finding path in floor " << floor_id << " from " << coords_[g_.u(e)] << " to " << coords_[g_.v(e)] << std::endl;
          best_path_[e] = id_to_floor_[floor_id].find_path(
              coords_[g_.u(e)], coords_[g_.v(e)]);
          last_timeout_[e] = t;
          if (best_path_[e]) {
            best_path_length_[e] = best_path_[e]->length();
          } else {
            usable[e] = false;
            solved = false;
          }
        }
      }
      if (solved) return dijk_shortest_path;
    }
    t *= 2; // TODO: get this param from somewhere
    k *= 2;
  }
  return dijk_shortest_path;
}

} // namespace mfplan
