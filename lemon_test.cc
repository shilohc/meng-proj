#include <iostream>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/dim2.h>

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

  return 0;
}
