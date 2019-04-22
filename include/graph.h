#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include <queue>
#include <set>
#include <map>

struct point
{
  int x;
  int y;
};

struct line
{
  point pt_a;
  point pt_b;
  int hv; // h 0, v 1
  int layer;
};

// TODO: rename this to avoid confuse with Edge
struct edge
{
  int index;
  int layer;
  int dist;

  int shape_a;
  int shape_b;

  point shape_a_pt;
  point shape_b_pt;

  int group_a;
  int group_b;

  std::vector<point> pts;
  std::vector<line> lines;
};

class Edge;


class Node
{
public:

  Node(const int& name);

  ~Node();

  bool operator< (const Node& n) const
  {
    return (this->key < n.key);
  }

  point getLoc() const;

  std::vector<Edge*> edges;

  int id;
  int key;
  int prev_i; // edge index connect to previous node
  point loc_;
  Node* prev;
  bool visited;

  // A* parameter
  int G;
  int H;
  int F;
};

class Edge
{
public:

  Edge(Node* a, Node* b, const int& w, const int& i);

  Node* getNeighbor(Node* n);

  Node* node[2];

  int index;
  int weight;
};

bool operator < (const Node& a, const Node& b);

bool operator == (const point& a, const point& b);

// used by the priority queue in MST to extract the min key
class CompareWeight
{
public:
   bool operator() (Node* lhs, Node* rhs)
   {
     return (lhs->key > rhs->key);
   }
};

class Graph
{
public:

  Graph(const std::vector<edge>& e);

  ~Graph();

  void addEdge(const int& v1, const int& v2, const int& w, const int& i);

  bool solveMST();

  bool solveAStar(point start, point goal);

  std::map<int, Node*> getResult();

  std::vector<edge> getEdges();

  std::vector<Edge*> edges;

  std::vector<Node*> nodes;

  void test();

private:

  bool findNodeInSet(const Node& n, const std::set<Node>& pq);

  bool isNeighbor();

  int getDist(const point& lhs, const point& rhs);

  std::map<int, Node*> nodes_map_;
};

