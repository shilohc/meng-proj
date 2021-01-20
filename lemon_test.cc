#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/dim2.h>
#include <lemon/dijkstra.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <opencv2/opencv.hpp>

using namespace lemon;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void viz_path(std::optional<og::PathGeometric> path, cv::Mat img) {
  if (!path.has_value()) { return; }
  cv::Scalar color(255, 0, 0);
  for (std::size_t i = 0; i < path->getStateCount(); ++i) {
    double x = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getX();
    double y = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getY();
    cv::Point ctr(x, y);
    cv::circle(img, ctr, 2, color, -1);
  }
}

void viz_point(dim2::Point<double> coords, cv::Mat img) {
  cv::Scalar color(0, 255, 0);
  cv::Point ctr(coords.x, coords.y);
  cv::circle(img, ctr, 5, color, -1);
}

// convert numeric image type to string for debug
std::string getImgType(int imgTypeInt) {
  // 7 base types, with five channel options each (none or C1, ..., C4)
  int numImgTypes = 35;

  int enum_ints[] = {
    CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
    CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
    CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
    CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
    CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
    CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
    CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

  std::string enum_strings[] = {
    "CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
    "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
    "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
    "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
    "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
    "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
    "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

  for (int i=0; i<numImgTypes; i++) {
    if (imgTypeInt == enum_ints[i]) return enum_strings[i];
  }
  return "unknown image type";
}

class Map2DValidityChecker : public ob::StateValidityChecker {
 public:
  Map2DValidityChecker(const ob::SpaceInformationPtr &space_info, cv::Mat map) :
      ob::StateValidityChecker(space_info), map(map) {
    max_width = map.cols - 1;
    max_height = map.rows - 1;
  }

  bool isValid(const ob::State* state) const override {
    const auto& se2_state = state->as<ob::SE2StateSpace::StateType>();

    const int x = std::clamp((int)se2_state->getX(), 0, max_width);
    const int y = std::clamp((int)se2_state->getY(), 0, max_height);
    int val = map.at<uchar>(x, y);

    //std::cout << "(" << x << ", " << y << "): " << val << std::endl;
    return (val > 210);
  }

 private:
  cv::Mat map;
  int max_width;
  int max_height;
};

class Floor {
 public:
  Floor() = default;

  Floor(const cv::Mat map, const int id) : map(map), id(id) {
    space = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0.0);
    bounds.setHigh(0, map.cols);
    bounds.setHigh(1, map.rows);
    space->setBounds(bounds);

    space_info = std::make_shared<ob::SpaceInformation>(space);

    auto validity_checker = std::make_shared<Map2DValidityChecker>(
        space_info, map);
    space_info->setStateValidityChecker(validity_checker);
  }

  std::optional<og::PathGeometric> find_path(
      dim2::Point<double> start_coords, dim2::Point<double> goal_coords) {
    ob::ScopedState<> start_state(space);
    start_state[0] = start_coords.x;
    start_state[1] = start_coords.y;
    ob::ScopedState<> goal_state(space);
    goal_state[0] = goal_coords.x;
    goal_state[1] = goal_coords.y;

    auto problem_def = std::make_shared<ob::ProblemDefinition>(space_info);
    problem_def->setStartAndGoalStates(start_state, goal_state);

    auto planner = std::make_shared<og::RRTstar>(space_info);
    //auto planner = std::make_shared<og::RRT>(space_info);
    //auto planner = std::make_shared<og::PRMstar>(space_info);
    planner->setProblemDefinition(problem_def);
    //planner->setRange(20.0);
    planner->setup();

    space_info->printSettings(std::cout);
    problem_def->print(std::cout);

    ob::PlannerStatus solved = planner->ob::Planner::solve(5.0);
    std::optional<og::PathGeometric> path;

    std::cout << "solved: " << solved << std::endl;
    if (bool(solved)) {
      ob::PathPtr p = problem_def->getSolutionPath();
      path = *(p->as<og::PathGeometric>());
      //path->interpolate();
      cv::Mat map_img;
      map.copyTo(map_img);
      viz_point(start_coords, map_img);
      viz_point(goal_coords, map_img);
      viz_path(path, map_img);
      cv::imwrite("test_out.png", map_img);
    }
    return path;
  }

 private:
  int id;
  cv::Mat map;
  std::shared_ptr<ob::SE2StateSpace> space;
  std::shared_ptr<ob::SpaceInformation> space_info;
};

int main(int argc, char** argv) {
  ListGraph g;
  
  ListGraph::NodeMap<int> floor_id(g);
  ListGraph::NodeMap<dim2::Point<double>> coords(g);
  ListGraph::EdgeMap<int> length(g);
  ListGraph::EdgeMap<bool> between_floor(g);
  std::string title;

  graphReader(g, "test_map.lgf")
    .nodeMap("floorid", floor_id)
    .nodeMap("coords", coords)
    .edgeMap("length", length)
    .edgeMap("between_floor", between_floor)
    .attribute("caption", title)
    .run();

  std::cout << countArcs(g) << std::endl;
  std::cout << countEdges(g) << std::endl;
  ListGraph::Node node3 = g.nodeFromId(3);
  std::cout << floor_id[node3] << std::endl;
  std::cout << coords[node3] << std::endl;
  for (ListGraph::IncEdgeIt a(g, node3); a != INVALID; ++a) {
    std::cout << between_floor[a] << std::endl;
  }
  std::cout << title << std::endl;

  // map files are all in current directory and have name format
  // test_map_[floor_id].png

  //std::unordered_map<int, cv::Mat> id_to_map;
  std::unordered_map<int, Floor> id_to_floor;
  std::array<int, 3> floor_ids = {0, 1, 2};
  std::string filename;
  for (int id : floor_ids) {
    filename = "test_map_" + std::to_string(id) + ".png";
    // TODO: test map first to make sure it is nonempty
    //id_to_map[id] = cv::imread(filename);
    id_to_floor[id] = Floor(cv::imread(filename), id);
  }

  /*
  for (ListGraph::NodeIt n(g); n != INVALID; ++n) {
    cv::Mat map_img;
    id_to_map[floor_id[n]].copyTo(map_img);
    cv::Scalar color(0, 255, 0);
    cv::Point ctr(coords[n].x, coords[n].y);
    cv::circle(map_img, ctr, 5, color, -1);
    cv::imwrite("test_out_" + std::to_string(g.id(n)) + ".png", map_img);
  }
  */

  ListGraph::NodeMap<int> dist(g);
  ListGraph::Node start = g.nodeFromId(1);
  ListGraph::Node end = g.nodeFromId(3);
  //dijkstra(g, length).distMap(dist).run(start, end);
  dijkstra(g, length).distMap(dist).run(end);
  std::cout << dist[start] << std::endl;

  id_to_floor[0].find_path(coords[g.nodeFromId(0)], coords[g.nodeFromId(2)]);

  return 0;
}
