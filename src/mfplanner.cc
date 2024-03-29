#include <cmath>
#include <limits>
#include <stdexcept>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

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
  bounds.setHigh(0, map_.cols);
  bounds.setHigh(1, map_.rows);
  space_->setBounds(bounds);

  // set longest valid segment length to 4 pixels
  space_->setLongestValidSegmentFraction(4/std::fmax(map_.rows, map_.cols));

  space_info_ = std::make_shared<ob::SpaceInformation>(space_);

  auto validity_checker = std::make_shared<Map2DValidityChecker>(
      space_info_, map_);
  space_info_->setStateValidityChecker(validity_checker);
  space_info_->setup();
}

StatusOrPath Floor::find_path(
    dim2::Point<double> start_coords, dim2::Point<double> goal_coords,
    double timeout, bool use_rrt_connect) {
  // TODO: it would probably make sense to split this into two functions:
  // one to set up the planning problem and one to put more time into planning.
  // Will also need to be able to configure early-return mode vs. path-
  // optimization mode for the phase 2 implementation.
  // use_rrt_connect will also add complexity in reusing the planning problem
  // across phase 1 and phase 2; since RRT-Connect is not asymptotically optimal
  // it can't be used for phase 2.
  ob::ScopedState<> start_state(space_);
  start_state[0] = start_coords.x;
  start_state[1] = start_coords.y;
  ob::ScopedState<> goal_state(space_);
  goal_state[0] = goal_coords.x;
  goal_state[1] = goal_coords.y;

  // set a high cost threshold so RRT* will return immediately after finding
  // any valid path
  ob::OptimizationObjectivePtr objective(
      new ob::PathLengthOptimizationObjective(space_info_));
  objective->setCostThreshold(ob::Cost(std::numeric_limits<float>::max()));

  auto problem_def = std::make_shared<ob::ProblemDefinition>(space_info_);
  problem_def->setStartAndGoalStates(start_state, goal_state);
  problem_def->setOptimizationObjective(objective);

  std::shared_ptr<ob::Planner> planner;
  if (use_rrt_connect) {
    planner = std::make_shared<og::RRTConnect>(space_info_);
  } else {
    planner = std::make_shared<og::RRTstar>(space_info_);
  }
  planner->setProblemDefinition(problem_def);
  planner->setup();

  ob::PlannerStatus solved = planner->ob::Planner::solve(timeout);
  std::optional<og::PathGeometric> path;

  if (bool(solved)) {
    ob::PathPtr p = problem_def->getSolutionPath();
    path = *(p->as<og::PathGeometric>());
    path->interpolate();
  }
  return std::make_tuple(solved, path);
}

void Floor::viz_coords(dim2::Point<double> coords) {
  cv::Scalar color(0, 255, 0);
  cv::Point ctr(coords.x, coords.y);
  cv::circle(map_img_, ctr, 5, color, -1);
  cv::imwrite("output/test_out_" + std::to_string(id_) + ".png", map_img_);
}

void Floor::viz_path(std::optional<og::PathGeometric>& path) {
  if (!path.has_value()) { return; }
  cv::Scalar color(255, 0, 0);
  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    double x = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getX();
    double y = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getY();
    cv::Point ctr(x, y);
    cv::circle(map_img_, ctr, 2, color, -1);
  }
  cv::imwrite("output/test_out_" + std::to_string(id_) + ".png", map_img_);
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
    auto img = cv::imread(it->second);
    if (img.empty()) {
      std::cout << "Could not find image file at " << it->second << std::endl;
      throw std::invalid_argument("image file not found");
    }
    id_to_floor_[it->first] = Floor(img, it->first);
  }

  // Add within-floor edges with appropriately populated length
  // Between-floor edges should have length pre-populated (or zero) in LGF file.
  // Not sure what will happen otherwise, but it probably won't be good.
  for (ListGraph::NodeIt n1(g_); n1 != INVALID; ++n1) {
    for (ListGraph::NodeIt n2(g_); n2 != INVALID; ++n2) {
      // would like to set this to std::next(n1) but std::next doesn't
      // work on NodeIts for some reason
      if ((floor_id_[n1] == floor_id_[n2]) and (n1 != n2)) {
        auto e = g_.addEdge(n1, n2);
        between_floor_[e] = false;
        length_[e] = euclidean_dist(e);
      }
    }
  }

  for (ListGraph::EdgeIt e(g_); e != INVALID; ++e) {
    last_timeout_[e] = 0;
    best_path_length_[e] = std::numeric_limits<float>::max();
  }
}

double MFPlanner::euclidean_dist(ListGraph::Edge e) {
  return std::sqrt((coords_[g_.u(e)] - coords_[g_.v(e)]).normSquare());
}

EdgeList MFPlanner::get_solution_path(
    CoordsAndFloor start, CoordsAndFloor goal, bool use_rrt_connect,
    double t_0, double k_0, double t_mult, double k_mult) {
  // Does phase 1 only.

  // Add start and goal as nodes
  ListGraph::Node start_node = g_.addNode();
  ListGraph::Node goal_node = g_.addNode();
  coords_[start_node] = std::get<0>(start);
  coords_[goal_node] = std::get<0>(goal);
  floor_id_[start_node] = std::get<1>(start);
  floor_id_[goal_node] = std::get<1>(goal);

  // add edges from start and goal to all other nodes in their floors
  for (ListGraph::NodeIt n(g_); n != INVALID; ++n) {
    if (n == start_node or n == goal_node) continue;
    if (floor_id_[n] == floor_id_[start_node]) g_.addEdge(start_node, n);
    if (floor_id_[n] == floor_id_[goal_node]) g_.addEdge(goal_node, n);
  }

  ListGraph::EdgeMap<double> cost(g_);
  ListGraph::EdgeMap<bool> usable(g_);
  for (ListGraph::EdgeIt e(g_); e != INVALID; ++e) {
    usable[e] = true;
    best_path_length_[e] = std::numeric_limits<float>::max();
    cost[e] = length_[e];
  }

  std::tuple<EdgeList, double> dijk = dijkstra(g_, cost, start_node, goal_node);
  EdgeList dijk_shortest_path = std::get<0>(dijk);
  double dijk_min_path_cost = std::get<1>(dijk) + 0.01;
  double l = dijk_min_path_cost;
  if (dijk_min_path_cost == std::numeric_limits<float>::max()) {
    std::cout << "No path between start and goal floors!" << std::endl;
    return dijk_shortest_path;
  }

  bool solved = false;

  double t = t_0;
  double k = k_0;

  while (!solved) {
    std::cout << "max l: " << k * dijk_min_path_cost << std::endl;
    while (l < k * dijk_min_path_cost) {
      for (ListGraph::EdgeIt e(g_); e != INVALID; ++e) {
        cost[e] = length_[e];
        if (best_path_[e].has_value()) cost[e] = best_path_length_[e];
        if (!usable[e]) cost[e] = std::numeric_limits<float>::max();
      }
      dijk = dijkstra(g_, cost, start_node, goal_node);
      dijk_shortest_path = std::get<0>(dijk);
      l = std::get<1>(dijk);
      std::cout << "new l: " << l << std::endl;
      solved = true;

      for (ListGraph::Edge e : dijk_shortest_path) {
        if (!between_floor_[e] && !best_path_[e].has_value()) {
          int floor_id = floor_id_[g_.u(e)];
          std::cout << "finding path in floor " << floor_id << " from " << coords_[g_.u(e)] << " to " << coords_[g_.v(e)] << std::endl;
          std::cout << t << std::endl;
          id_to_floor_[floor_id].viz_coords(coords_[g_.u(e)]);
          id_to_floor_[floor_id].viz_coords(coords_[g_.v(e)]);

          auto status_or_path = id_to_floor_[floor_id].find_path(
              coords_[g_.u(e)], coords_[g_.v(e)], t, use_rrt_connect);
          auto planner_status = std::get<0>(status_or_path);

          // make sure to update both e and its reverse edge because
          // a LEMON ListGraph is just a directed graph in a funny hat
          for (ConEdgeIt<ListGraph> f(g_, g_.u(e), g_.v(e)); f!=INVALID; ++f) {
            best_path_[f] = std::get<1>(status_or_path);
            last_timeout_[f] = t;
            if (planner_status == ob::PlannerStatus::EXACT_SOLUTION) {
              best_path_length_[f] = best_path_[f]->length();
              id_to_floor_[floor_id].viz_path(best_path_[f]);
            } else {
              // TODO: somehow unusable edges are getting used anyway and
              // it's screwing up sometimes on the stata split elevator map
              // with RRT* but not RRT-Connect.  why
              // it might be fixed now, but it also might not be.
              // update: it's not fixed :(
              std::cout << "edge not usable" << std::endl;
              usable[e] = false;
              usable[f] = false;
              best_path_[e].reset();
              best_path_[f].reset();
              best_path_length_[e] = 0.0;
              best_path_length_[f] = 0.0;
              solved = false;
            }
          }

          auto status = std::get<0>(status_or_path);
          if (status == ob::PlannerStatus::StatusType::INVALID_START) {
            std::cout << "invalid start state!!" << std::endl;
            g_.erase(g_.u(e));
          }
          if (status == ob::PlannerStatus::StatusType::INVALID_GOAL) {
            std::cout << "invalid goal state!!" << std::endl;
            g_.erase(g_.v(e));
          }
        }
      }
      if (solved) return dijk_shortest_path;
    }
    t *= t_mult;
    k *= k_mult;
    for (ListGraph::EdgeIt e(g_); e != INVALID; ++e) {
      usable[e] = true;
    }
  }

  return dijk_shortest_path;
}

void MFPlanner::print_edges(EdgeList edges) {
  for (ListGraph::Edge e : edges) {
    if (between_floor_[e]) {
      std::cout << "Take edge " << g_.id(e) << " between floor "
        << floor_id_[g_.u(e)] << " and floor " << floor_id_[g_.v(e)] 
        << std::endl;
    } else {
      std::cout << "Travel through floor " << floor_id_[g_.u(e)] << " from " << coords_[g_.u(e)] << " to " << coords_[g_.v(e)] << std::endl;
    }
  }
}

} // namespace mfplan
