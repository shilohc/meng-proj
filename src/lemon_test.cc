#include <algorithm>
#include <chrono>
#include <ctime>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/dim2.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "mfplan/dijkstra.h"
#include "mfplan/mfplanner.h"

using namespace lemon;
namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(int argc, char** argv) {
  ListGraph g;
  
  ListGraph::NodeMap<int> floor_id(g);
  ListGraph::NodeMap<dim2::Point<double>> coords(g);
  ListGraph::EdgeMap<double> length(g);
  ListGraph::EdgeMap<bool> between_floor(g);
  ListGraph::EdgeMap<bool> cost(g);
  std::string title;

  // TODO: read in a metadata yaml file with the graph and map filenames
  graphReader(g, "test_map.lgf")
    .nodeMap("floorid", floor_id)
    .nodeMap("coords", coords)
    .edgeMap("length", length)
    .edgeMap("between_floor", between_floor)
    .attribute("caption", title)
    .run();

  /*
  ListGraph::Node start = g.addNode();
  ListGraph::Node goal = g.addNode();
  floor_id[start] = 1;
  floor_id[goal] = 2;
  coords[start] = dim2::Point(50, 50);
  coords[goal] = dim2::Point(50, 50);
  g.addEdge(start, g.nodeFromId(1));
  g.addEdge(goal, g.nodeFromId(3));
  */

  // map files are all in current directory and have name format
  // test_map_[floor_id].png

  /*
  std::unordered_map<int, mfplan::Floor> id_to_floor;
  std::array<int, 3> floor_ids = {0, 1, 2};
  std::string filename;
  for (int id : floor_ids) {
    filename = "test_map_" + std::to_string(id) + ".png";
    // TODO: test map first to make sure it is nonempty
    id_to_floor[id] = mfplan::Floor(cv::imread(filename), id);
  }
  */

  /*
  // visualize doorways
  for (ListGraph::NodeIt n(g); n != INVALID; ++n) {
    id_to_floor[floor_id[n]].viz_coords(coords[n]);
  }
  */

  mfplan::CoordsAndFloor start = std::make_tuple(dim2::Point(300, 300), 1);
  //mfplan::CoordsAndFloor goal = std::make_tuple(dim2::Point(100, 100), 2);
  //mfplan::CoordsAndFloor goal = std::make_tuple(dim2::Point(110, 110), 11);
  mfplan::CoordsAndFloor goal = std::make_tuple(dim2::Point(110, 330), 191);

  /*
  std::unordered_map<int, std::string> map_files {
      {0, "test_map_0.png"},
      {1, "test_map_1.png"},
      {2, "test_map_2.png"}};
  auto mfplanner = mfplan::MFPlanner("test_map.lgf", map_files);
  */
  std::unordered_map<int, std::string> map_files {
      {1, "mit_basement_map/1_0.png"},
      {2, "mit_basement_map/2_0.png"},
      {3, "mit_basement_map/3_0.png"},
      {4, "mit_basement_map/4_0.png"},
      {5, "mit_basement_map/5_0.png"},
      {6, "mit_basement_map/6_0.png"},
      {62, "mit_basement_map/6C_0.png"},
      {7, "mit_basement_map/7_0.png"},
      {8, "mit_basement_map/8_0.png"},
      {9, "mit_basement_map/9_0.png"},
      {10, "mit_basement_map/10_0.png"},
      {11, "mit_basement_map/11_0.png"},
      {13, "mit_basement_map/13_0.png"},
      {14, "mit_basement_map/14_0.png"},
      {16, "mit_basement_map/16_0.png"},
      {18, "mit_basement_map/18_0.png"},
      {26, "mit_basement_map/26_0.png"},
      {32, "mit_basement_map/32_0.png"},
      {36, "mit_basement_map/36_0M.png"},
      {54, "mit_basement_map/54_00.png"},
      {56, "mit_basement_map/56_0.png"},
      {66, "mit_basement_map/66_0.png"},
      {68, "mit_basement_map/68_0.png"},
      {76, "mit_basement_map/76_0.png"},
      {171, "mit_basement_map/E17_0.png"},
      {181, "mit_basement_map/E18_0.png"},
      {191, "mit_basement_map/E19_0.png"}};
  auto mfplanner = mfplan::MFPlanner("mit_basement_map/mit_basement_map.lgf",
      map_files);

  auto start_time = std::chrono::system_clock::now();
  mfplan::EdgeList path = mfplanner.get_solution_path(start, goal);
  auto end_time = std::chrono::system_clock::now();

  std::chrono::duration<double> elapsed_seconds = end_time - start_time;
  std::cout << "solution path found after " << elapsed_seconds.count()
            << " seconds" << std::endl;

  auto composite_map = mfplan::Floor(cv::imread("mit_basement_map/composite_map.png"), 0);
  start_time = std::chrono::system_clock::now();
  auto status_or_path = composite_map.find_path(dim2::Point(603, 6285), dim2::Point(10950, 1555), 800.0);
  end_time = std::chrono::system_clock::now();
  elapsed_seconds = end_time - start_time;
  auto best_path = std::get<1>(status_or_path);
  if (best_path) {
    composite_map.viz_path(best_path);
  }
  std::cout << "composite map solution found after " << elapsed_seconds.count()
            << " seconds" << std::endl;

  return 0;
}
