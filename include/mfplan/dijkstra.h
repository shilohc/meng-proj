#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include <lemon/list_graph.h>

// stands for multi-floor planner
namespace mfplan {

class DijkComparator {
 public:
  DijkComparator(lemon::ListGraph::NodeMap<double>& cost, lemon::ListGraph& g);
  bool operator() (const lemon::ListGraph::Node& n1,
                   const lemon::ListGraph::Node& n2);
 private:
  lemon::ListGraph& g_;
  lemon::ListGraph::NodeMap<double>& cost_;
};

// priority queue that contains ListGraph nodes and is prioritized by the
// ListGraph::NodeMap<double> dist
typedef std::priority_queue<lemon::ListGraph::Node,
        std::vector<lemon::ListGraph::Node>,
        DijkComparator> DijkPQ;

typedef std::vector<lemon::ListGraph::Edge> EdgeList;

std::tuple<EdgeList, double> dijkstra(lemon::ListGraph& g,
    lemon::ListGraph::EdgeMap<double>& cost,
    lemon::ListGraph::Node& start, lemon::ListGraph::Node& goal);

} // namespace mfplan

#endif
