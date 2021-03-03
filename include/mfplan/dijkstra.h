#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <queue>
#include <set>
#include <tuple>
#include <vector>

#include <lemon/list_graph.h>

using namespace lemon;

// stands for multi-floor planner
namespace mfplan {

class DijkComparator {
 public:
  DijkComparator(ListGraph::NodeMap<double>& cost, ListGraph& g) :
    cost_(cost), g_(g) {}
  bool operator() (const ListGraph::Node& n1, const ListGraph::Node& n2) {
    return ((cost_[n1] > cost_[n2]) ||
            (cost_[n1] == cost_[n2] && g_.id(n1) > g_.id(n2)));
  }
 private:
  ListGraph& g_;
  ListGraph::NodeMap<double>& cost_;
};

// priority queue that contains ListGraph nodes and is prioritized by the
// ListGraph::NodeMap<double> dist
typedef std::priority_queue<ListGraph::Node, std::vector<ListGraph::Node>,
        DijkComparator> dijk_pq;

std::vector<ListGraph::Edge> dijkstra(ListGraph& g,
    ListGraph::EdgeMap<double>& cost,
    ListGraph::Node& start, ListGraph::Node& goal) {
  ListGraph::NodeMap<double> dist(g);
  for (ListGraph::NodeIt n(g); n != INVALID; ++n) {
    dist[n] = std::numeric_limits<double>::max();
  }
  dist[start] = 0.0;
  
  ListGraph::NodeMap<int> parent_edge_id(g);
  ListGraph::NodeMap<ListGraph::Edge> parent_edge(g);
  std::set<ListGraph::Node> visited;
  dijk_pq frontier(DijkComparator(dist, g));
  frontier.push(start);

  while ((visited.count(goal) == 0) && (!frontier.empty())) {
    ListGraph::Node current = frontier.top();
    frontier.pop();
    if (visited.count(current) > 0) continue;
    ListGraph::Node adj;
    for (ListGraph::OutArcIt e(g, current); e != INVALID; ++e) {
      adj = g.oppositeNode(current, e);
      if (visited.count(adj) > 0) continue;
      if (dist[current] + cost[e] < dist[adj]) {
        dist[adj] = dist[current] + cost[e];
        parent_edge_id[adj] = g.id(e);
        parent_edge[adj] = e;
      }
      frontier.push(adj);
    }
    visited.insert(current);
  }

  std::vector<ListGraph::Edge> solution_path;
  if (visited.count(goal)) {
    ListGraph::Node current = goal;
    while (current != start && g.id(current) != -1) {
      solution_path.push_back(parent_edge[current]);
      current = g.oppositeNode(current, parent_edge[current]);
    }
  }
  return solution_path;
}

} // namespace mfplan

#endif
