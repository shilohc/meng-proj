#include <iostream>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/dim2.h>
#include <lemon/dijkstra.h>

#include <opencv2/opencv.hpp>

using namespace lemon;

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

  std::unordered_map<int, cv::Mat> id_to_map;
  std::array<int, 3> floor_ids = {0, 1, 2};
  std::string filename;
  for (int id : floor_ids) {
    filename = "test_map_" + std::to_string(id) + ".png";
    id_to_map[id] = cv::imread(filename);
  }

  for (ListGraph::NodeIt n(g); n != INVALID; ++n) {
    cv::Mat map_img;
    id_to_map[floor_id[n]].copyTo(map_img);
    cv::Scalar color(0, 255, 0);
    cv::Point ctr(coords[n].x, coords[n].y);
    cv::circle(map_img, ctr, 5, color, -1);
    cv::imwrite("test_out_" + std::to_string(g.id(n)) + ".png", map_img);
  }

  ListGraph::NodeMap<int> dist(g);
  ListGraph::Node start = g.nodeFromId(1);
  ListGraph::Node end = g.nodeFromId(3);
  //dijkstra(g, length).distMap(dist).run(start, end);
  dijkstra(g, length).distMap(dist).run(end);
  std::cout << dist[start] << std::endl;

  return 0;
}
