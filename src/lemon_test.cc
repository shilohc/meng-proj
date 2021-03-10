#include <algorithm>
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

#include "mfplan/cv_utils.h"
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

  ListGraph::Node start = g.addNode();
  ListGraph::Node goal = g.addNode();
  floor_id[start] = 1;
  floor_id[goal] = 2;
  coords[start] = dim2::Point(50, 50);
  coords[goal] = dim2::Point(50, 50);
  g.addEdge(start, g.nodeFromId(1));
  g.addEdge(goal, g.nodeFromId(3));

  // map files are all in current directory and have name format
  // test_map_[floor_id].png

  std::unordered_map<int, mfplan::Floor> id_to_floor;
  std::array<int, 3> floor_ids = {0, 1, 2};
  std::string filename;
  for (int id : floor_ids) {
    filename = "test_map_" + std::to_string(id) + ".png";
    // TODO: test map first to make sure it is nonempty
    id_to_floor[id] = mfplan::Floor(cv::imread(filename), id);
  }

  /*
  // visualize doorways
  for (ListGraph::NodeIt n(g); n != INVALID; ++n) {
    id_to_floor[floor_id[n]].viz_coords(coords[n]);
  }
  */

  /*
  std::vector<ListGraph::Edge> path = mfplan::dijkstra(g, length, start, goal);
  std::cout << "dijkstra path length " << path.size() << std::endl;
  for (ListGraph::Edge e : path) {
    std::cout << "u: " << g.id(g.u(e)) << " v: " << g.id(g.v(e)) << std::endl;
  }
  */

  //id_to_floor[0].find_path(coords[g.nodeFromId(0)], coords[g.nodeFromId(2)]);

  return 0;
}
