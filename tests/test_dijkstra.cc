#include <gtest/gtest.h>

#include <lemon/list_graph.h>
#include <lemon/lgf_reader.h>

#include "mfplan/dijkstra.h"

using namespace lemon;

namespace {

TEST(DijkstraTest, SimpleTestMap) {
  ListGraph g;
  ListGraph::EdgeMap<double> length(g);

  graphReader(g, "test_map.lgf").edgeMap("length", length).run();

  ListGraph::Node start = g.addNode();
  ListGraph::Node goal = g.addNode();
  g.addEdge(start, g.nodeFromId(1));
  g.addEdge(goal, g.nodeFromId(3));

  std::tuple<mfplan::EdgeList, double> dijk = mfplan::dijkstra(
      g, length, start, goal);
  mfplan::EdgeList path = std::get<0>(dijk);
  double cost = std::get<1>(dijk);

  EXPECT_EQ(path.size(), 5);
}

// TODO: add a unit test with a more complex input graph

} // namespace

