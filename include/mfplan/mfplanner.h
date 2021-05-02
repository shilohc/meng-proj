#ifndef MFPLANNER_H
#define MFPLANNER_H

#include <unordered_map>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/dim2.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <opencv2/opencv.hpp>

#include "mfplan/dijkstra.h"

namespace mfplan {

typedef std::tuple<ompl::base::PlannerStatus,
        std::optional<ompl::geometric::PathGeometric>> StatusOrPath;

class Map2DValidityChecker : public ompl::base::StateValidityChecker {
 public:
  Map2DValidityChecker(const ompl::base::SpaceInformationPtr &space_info,
      cv::Mat map) : ompl::base::StateValidityChecker(space_info), map_(map) {}

  bool isValid(const ompl::base::State* state) const override;

 private:
  cv::Mat map_;
};

class Floor {
 public:
  Floor() = default;
  Floor(const cv::Mat map, const int id);
  StatusOrPath find_path(
      lemon::dim2::Point<double> start_coords,
      lemon::dim2::Point<double> goal_coords,
      double timeout=5.0);
  void viz_coords(lemon::dim2::Point<double> coords);
  void viz_path(std::optional<ompl::geometric::PathGeometric>& path);

 private:
  int id_;
  cv::Mat map_;
  cv::Mat map_img_;
  std::shared_ptr<ompl::base::SE2StateSpace> space_;
  std::shared_ptr<ompl::base::SpaceInformation> space_info_;
};

typedef std::tuple<lemon::dim2::Point<double>, int> CoordsAndFloor;

class MFPlanner {
 public:
  MFPlanner(const std::string& graph_file,
      const std::unordered_map<int, std::string>& map_files);

  EdgeList get_solution_path(CoordsAndFloor start, CoordsAndFloor goal,
      double t_0=0.5, double k_0=2, double t_mult=2, double k_mult=2);
  double euclidean_dist(lemon::ListGraph::Edge e);
  void print_edges(EdgeList edges);

 private:
  lemon::ListGraph g_;
  lemon::ListGraph::NodeMap<int> floor_id_;
  lemon::ListGraph::NodeMap<lemon::dim2::Point<double>> coords_;
  lemon::ListGraph::EdgeMap<double> length_; // lower bound length
  lemon::ListGraph::EdgeMap<bool> between_floor_;
  lemon::ListGraph::EdgeMap<std::optional< // optional is initialized empty
    ompl::geometric::PathGeometric>> best_path_;
  lemon::ListGraph::EdgeMap<double> best_path_length_; // length of best_path_
  lemon::ListGraph::EdgeMap<double> last_timeout_;

  std::unordered_map<int, Floor> id_to_floor_;
};

} // namespace mfplan

#endif
