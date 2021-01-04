#include <algorithm>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

#include <opencv2/opencv.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;
/*
//namespace se2 = ompl::base::SE2StateSpace;
using se2 = ob::SE2StateSpace;
using SE2State = ob::ScopedState<ob::SE2StateSpace::StateType>;
// TODO: oh my god I am losing my fucking mind over the SE2State type.
// there are so many compiler errors.  what the fuck.
// i am THIS close to defining a nice struct or whatever with overloaded
// + and -.  it JUST needs to hold x, y, and yaw that's ALL

class Region {
 public:
  //Region(const cv::Mat& map, double radius, se2::StateType origin) :
  Region(const cv::Mat& map, double radius, SE2State origin) :
    map(map), radius(radius), origin(origin) {

    space = std::make_shared<ob::SE2StateSpace>();

    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, origin.getX() - 2 * radius);
    bounds.setLow(1, origin.getY() - 2 * radius);
    bounds.setHigh(0, origin.getX() + 2 * radius);
    bounds.setHigh(1, origin.getY() + 2 * radius);
    space->setBounds(bounds);

    space_info = std::make_shared<ob::SpaceInformation>(space);

    // TODO: create RegionValidityChecker
    //auto validity_checker = std::make_shared<RegionValidityChecker>(
    //    space_info, map, origin, bounds);
    //space_info->setStateValidityChecker(validity_checker);
    space_info->setStateValidityChecker(this->isValid);
  }

  // states are defined wrt the region reference frame so the state
  // validity checker will need to convert to the map frame
  // but this is an implementation detail of Region and users of Region
  // shouldn't need to know about the map
  
  bool isValid(const ob::State* state) const {
    const auto& se2_state = state->as<ob::SE2StateSpace::StateType>();
    // Yaw is not relevant for state validity since we assume a point robot
    const int x = se2_state->getX() + origin.getX();
    const int y = se2_state->getY() + origin.getY();
    return map.at<uchar>(y, x) > 200;
  }

  //const ob::SpaceInformationPtr& space_info;
  std::shared_ptr<ob::SpaceInformation> space_info;
  SE2State origin; // wrt map reference frame
  double radius; // in pixels

 private:
  const cv::Mat& map;
  std::shared_ptr<ob::SE2StateSpace> space;
};

// TODO: can I define these so that they override - and + on StateTypes?
SE2State add_states(SE2State s1, SE2State s2) {
  SE2State out;
  out.setX(s1.getX() + s2.getX());
  out.setY(s1.getY() + s2.getY());
  out.setYaw(s1.getYaw() + s2.getYaw());
  return out;
}

SE2State negate(SE2State s) {
  SE2State out;
  out.setX(-1 * s.getX());
  out.setY(-1 * s.getY());
  out.setYaw(-1 * s.getYaw());
  return out;
}

class Edge {
 public:
  Edge(const Region& r1, const Region& r2, SE2State doorway) :
    r1(r1), r2(r2), doorway(doorway) {
    tf = add_states(r2, negate(r1));
  }

  SE2State doorwayWrtR2() {
    return add_states(doorway, tf);
  }

  // possibly this should just store region origin and ID instead of
  // the full region type?  although it's a reference so it shouldn't matter
  const Region& r1;
  // TODO: is r2 even necessary for anything?
  const Region& r2;
  SE2State tf; // tf from r1's origin to r2's origin
  SE2State doorway; // location of doorway relative to r1
};

// Usage for Region and Edge: Region path specifies a vector of regions and a
// vector of edges (or something).  For each region, generate a path either
// from the start state to the doorway on the next edge, the doorway on the
// prev edge to the doorway on the next edge, or the doorway on the prev edge
// to the end state.
std::vector<std::optional<og::PathGeometric>> find_path(
    std::vector<Edge> region_path,
    //const ob::SpaceInformationPtr& space_info,
    SE2State& start) {
  // start is assumed to be in map reference frame
  // TODO: space_info will be different for each region.  All start and goal
  // states should be RELATIVE TO THE REGION ORIGIN, and the space_info
  // refers to the region origin.

  std::vector<std::optional<og::PathGeometric>> total_path;
  
  // Last edge in region path should be a special "goal edge" that has its
  // doorway set to goal, so goal is not an input to this function

  // start state is assumed to be inside first region
  // transform local_start to first region's reference frame
  SE2State local_start = add_states(start, negate(path[0].r1.origin));
  SE2State local_goal;
  for (auto edge : region_path) {
    // TODO: get space_info from region r1
    auto problem_def = std::make_shared<ob::ProblemDefinition>(
        edge.r1.space_info);
    auto planner = std::make_shared<og::RRTstar>(edge.r1.space_info);
    // get pose of doorway relative to origin of first region in edge
    local_goal = edge.doorway;
    problem_def->setStartAndGoalStates(local_start, local_goal);
    planner->setProblemDefinition(problem_def);
    planner->setup();
    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    std::optional<og::PathGeometric> path;
    if (bool(solved)) {
      ob::PathPtr p = problem_def->getSolutionPath();
      path = *(p->as<og::PathGeometric>());
      path->interpolate();
      for (std::size_t i=0; i < path->getStateCount(); ++i) {
        // TODO: add points to map for visualization
        double x = path->getState(i)->as<ob::SE2StateSpace::StateType>()
                       ->getX();
        double y = path->getState(i)->as<ob::SE2StateSpace::StateType>()
                       ->getY();
        //std::cout << "(" << x << ", " << y << ")" << std::endl;
    }
    total_path.push_back(path);
    local_start = add_states(local_goal, edge.tf);
  }

  return total_path;

  // TODO: condense total path into a single PathGeometric (will need to be
  // transformed to some global reference frame, probably?)
  // TODO: what to do if one of the local paths is False?
}
*/

void viz_path(std::optional<og::PathGeometric> path, cv::Mat img) {
  if (!path.has_value()) { return; }
  cv::Scalar color(255, 0, 0);
  for (std::size_t i=0; i < path->getStateCount(); ++i) {
    double x = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getX();
    double y = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getY();
    cv::Point ctr(y, x);
    cv::circle(img, ctr, 2, color, -1);
  }
}

/*
void viz_region(Region r, cv::Mat img) {
  cv::Scalar color(0, 0, 255);
  double x = r.origin.getX();
  double y = r.origin.getY();
  cv::Point ctr(y, x);
  cv::circle(img, ctr, 2, color, -1);
  cv::circle(img, ctr, int(r.radius), color, 2);
}

void viz_point(SE2State point, cv::Mat img) {
  cv::Scalar color(0, 255, 0);
  double x = point.getX();
  double y = point.getY();
  cv::Point ctr(y, x);
  cv::circle(img, ctr, 5, color, -1);
}
*/

// Later on I'll also need a fast nearest-neighbors alg for picking the region
// closest to a given start/goal state.  

// take number image type number (from cv::Mat.type()), get OpenCV's enum string.
std::string getImgType(int imgTypeInt)
{
    int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

    int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
                             CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
                             CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
                             CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
                             CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
                             CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
                             CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

    std::string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
                             "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
                             "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                             "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                             "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                             "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                             "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

    for(int i=0; i<numImgTypes; i++)
    {
        if(imgTypeInt == enum_ints[i]) return enum_strings[i];
    }
    return "unknown image type";
}

class Map2DValidityChecker : public ob::StateValidityChecker{
 public:
  Map2DValidityChecker(const ob::SpaceInformationPtr &space_info, cv::Mat map) :
    ob::StateValidityChecker(space_info), map(map) {
      max_width = map.cols - 1;
      max_height = map.rows - 1;
    }

  // TODO: i think i am doing something wrong here?  or maybe the viz is
  // fucked up.  but i am seeing planned paths that intersect with obstacles.
  // TODO: should maybe be virtual?
  bool isValid(const ob::State* state) const override {
    const auto& se2_state = state->as<ob::SE2StateSpace::StateType>();

    // Yaw is not relevant for state validity since we assume a point robot
    const int x = std::clamp((int)se2_state->getX(), 0, max_width);
    const int y = std::clamp((int)se2_state->getY(), 0, max_height);
    //std::cout << getImgType(map.type()); // CV_8UC3
    /*
    if (int(map.at<uchar>(y, x)) != 205) {
      std::cout << "(" << x << ", " << y << "): ";
      std::cout << int(map.at<uchar>(y, x)) << std::endl;
    }*/

    return int(map.at<uchar>(y, x)) > 210;
  }

 private:
  cv::Mat map;
  int max_width;
  int max_height;
};

class Map2D {
 public:
  Map2D(const std::string& map_file) {
    // TODO: handle empty image files better
    map = cv::imread(map_file);
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
      ob::ScopedState<> start, ob::ScopedState<> goal) {
    auto problem_def = std::make_shared<ob::ProblemDefinition>(space_info);
    problem_def->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTstar>(space_info);
    planner->setProblemDefinition(problem_def);
    planner->setup();

    space_info->printSettings(std::cout);
    problem_def->print(std::cout);

    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);
    std::optional<og::PathGeometric> path;

    std::cout << "solved: " << solved << std::endl;
    // TODO: it'd be useful to viz path, start, and goal
    if (bool(solved)) {
      ob::PathPtr p = problem_def->getSolutionPath();
      path = *(p->as<og::PathGeometric>());
      path->interpolate();
      /*
      for (std::size_t i=0; i < path->getStateCount(); ++i) {
        // TODO: add points to map for visualization
        double x = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getX();
        double y = path->getState(i)->as<ob::SE2StateSpace::StateType>()->getY();
        std::cout << "(" << x << ", " << y << ")" << std::endl;
      }*/
      std::cout << "Visualizing path" << std::endl;
      cv::Mat map_img;
      map.copyTo(map_img);
      viz_path(path, map_img);
      //viz_point(start->as<se2::StateType>(), map_img);
      //viz_point(goal->as<se2::StateType>(), map_img);
      cv::imwrite("test_out.png", map_img);
    }
    return path;
  }

  std::optional<og::PathGeometric> find_random_path() {
    //ob::ScopedState<ob::SE2StateSpace::StateType> start(space);
    ob::ScopedState<> start(space);
    start.random();
    //start->setXY(169, 605);
    //start->setYaw(0);
    ob::ScopedState<> goal(space);
    goal.random();

    std::optional<og::PathGeometric> path = find_path(start, goal);
    std::cout << (path.has_value()) << std::endl;
    
    return path;
  }

 private:
  cv::Mat map;
  std::shared_ptr<ob::SE2StateSpace> space;
  std::shared_ptr<ob::SpaceInformation> space_info;
};

int main(int argc, char** argv) {
  // TODO: use csail maps
  const std::string map_file = "/Users/shiloh/Documents/mit/meng_proj/data/fetch_maps/908/merged_map.png";
  // test map to make sure it is nonempty
  //auto map = cv::imread(map_file);
  //std::cout << map.cols << std::endl;

  auto map_state_space = Map2D(map_file);
  std::cout << map_state_space.find_random_path().has_value() << std::endl;
  return 0;
}

