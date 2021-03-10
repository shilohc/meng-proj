#ifndef MFPLANNER_H
#define MFPLANNER_H

#include <unordered_map>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/dim2.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <opencv2/opencv.hpp>

namespace mfplan {

class Map2DValidityChecker : public ompl::base::StateValidityChecker {
 public:
  Map2DValidityChecker(const ompl::base::SpaceInformationPtr &space_info,
      cv::Mat map) : ompl::base::StateValidityChecker(space_info), map_(map) {}

  bool isValid(const ompl::base::State* state) const override {
    const auto& se2_state = state->as<ompl::base::SE2StateSpace::StateType>();

    const int x = int(se2_state->getX());
    const int y = int(se2_state->getY());

    if (!si_->satisfiesBounds(state)) {
      return false;
    }

    const int val = map_.at<uchar>(y, x);
    return (val > 210);
  }

 private:
  cv::Mat map_;
};

class Floor {
 public:
  Floor() = default;
  Floor(const cv::Mat map, const int id);
  std::optional<ompl::geometric::PathGeometric> find_path(
      lemon::dim2::Point<double> start_coords,
      lemon::dim2::Point<double> goal_coords);
  void viz_coords(lemon::dim2::Point<double> coords);

 private:
  int id_;
  cv::Mat map_;
  cv::Mat map_img_;
  std::shared_ptr<ompl::base::SE2StateSpace> space_;
  std::shared_ptr<ompl::base::SpaceInformation> space_info_;
};

class MFPlanner {
 public:
  MFPlanner(const std::string& graph_file,
      const std::unordered_map<int, std::string>& map_files);

  // TODO: make a struct that can hold start and goal -- coords wrt
  // floor map and floor id
  std::vector<lemon::ListGraph::Edge> get_solution_path();

 private:
  // TODO: add edge/node maps for all structures in pseudocode
  lemon::ListGraph g_;
  lemon::ListGraph::NodeMap<int> floor_id_;
  lemon::ListGraph::NodeMap<lemon::dim2::Point<double>> coords_;
  lemon::ListGraph::EdgeMap<double> length_;
  lemon::ListGraph::EdgeMap<bool> between_floor_;
  //lemon::ListGraph::EdgeMap<bool> cost_;
  //lemon::ListGraph::EdgeMap<> best_path_;

  std::unordered_map<int, Floor> id_to_floor_;
};

} // namespace mfplan

#endif
