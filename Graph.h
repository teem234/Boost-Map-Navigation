#include <iostream>
#include <deque>
#include <iterator>
#include <vector>
#include <fstream>
#include <string>

#include <boost/graph/adjacency_list.hpp>
#include "boost/graph/topological_sort.hpp"
#include "boost/graph/astar_search.hpp"
#include "boost/graph/dijkstra_shortest_paths.hpp"
#include "boost/functional/hash.hpp"

struct Vertex{
    char name;
    int visited;

    int x;
    int y;
};

struct EdgeProperties {
  double weight;

};
struct location
{
  float y, x; 
};

class endNode_visitor : public boost::default_bfs_visitor {
public:
    template <class Vertex, class Graph>
    void find_vertex(Vertex end, Graph &adjList)
    {
        std::cout<<"Found vertex " << end << std::endl;

    }
};
typedef float cost;
template <class myGraph, class CostType>
class distance_heuristic : public boost::astar_heuristic<myGraph, CostType>
{
public:
  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,  Vertex,boost::property<boost::edge_weight_t,cost > > mygraph_t;
  typedef typename boost::graph_traits<mygraph_t>::vertex_descriptor vertex_t;
  
  
  distance_heuristic(myGraph map , vertex_t goal)
    : m_map(map), m_goal(goal) {}
  CostType operator()(vertex_t u)
  {
    CostType dx = m_map[m_goal].x - m_map[u].x;
    CostType dy = m_map[m_goal].y - m_map[u].y;
    return ::sqrt(dx * dx + dy * dy);
  }
private:
  myGraph m_map;
  vertex_t m_goal;
};
template <class myGraph, class CostType>
class emptyHeuristic : public boost::astar_heuristic<myGraph, CostType>
{
public:
  typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,  Vertex,boost::property<boost::edge_weight_t,cost > > mygraph_t;
  typedef typename boost::graph_traits<mygraph_t>::vertex_descriptor vertex_t;
  
  
  emptyHeuristic(myGraph map , vertex_t goal)
    : m_map(map), m_goal(goal) {}
  CostType operator()(vertex_t u)
  {
    return 0;
  }
private:
  myGraph m_map;
  vertex_t m_goal;
};
struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,  Vertex,boost::property<boost::edge_weight_t,cost > > mygraph_t;
  typedef typename boost::graph_traits<mygraph_t>::vertex_descriptor vertex_t;
  astar_goal_visitor(vertex_t goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};

typedef boost::adjacency_list<boost::listS, boost::vecS,boost::undirectedS,boost::no_property,EdgeProperties> gGraph;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,  Vertex,boost::property<boost::edge_weight_t, cost> > mygraph_t;
typedef typename boost::graph_traits<mygraph_t>::vertex_descriptor vertex_t;
typedef std::pair<int, int> edge;
typedef boost::property_map < mygraph_t, boost::vertex_index_t >::type IndexMap;
typedef boost::property_map<mygraph_t, boost::edge_weight_t>::type WeightMap;
typedef boost::graph_traits<mygraph_t>::vertex_iterator vertex_iter;

class Graph{

public:
    WeightMap weightmap;
    Graph(std::string fileName);
    std::vector<long unsigned int> Dijkstras(int startX, int startY, int endX, int endY);
    std::vector<long unsigned int> AStar(int startX, int startY, int endX, int endY);
    std::vector<long unsigned int> BranchAndBound(int startX, int startY, int endX, int endY);
    std::vector<long unsigned int> BranchAndBoundExtended(int startX, int startY, int endX, int endY);
    std::vector<long unsigned int> BranchAndBoundHeuristic(int startX, int startY, int endX, int endY);
    void printCharMap();
    void generateBoostGraph();

private:
    void populateGraphVector();
    std::string fileName;
    mygraph_t adjList;
    std::vector<std::vector<char> > charMap;

    std::vector<std::vector<vertex_t> > vertexMap;
    std::vector<location> LocationMap;

};
