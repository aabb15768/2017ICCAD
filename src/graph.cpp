#include <graph.h>

Node::Node(const int& name):
  id(name),
  prev(nullptr),
  visited(false)
{

}

//bool operator<(const Node& a, const Node& b)
//{
//  return a.key < b.key;
//}

bool operator==(const point& a, const point& b)
{
  return (a.x == b.x)&&(a.y == b.y);
}

Node::~Node(){}

point Node::getLoc() const
{
  return loc_;
}

Edge::Edge(Node *a, Node *b, const int &w, const int &i):
  weight(w),
  index(i)
{
  if (a->id <= b->id)
  {
    node[0] = a;
    node[1] = b;
  }
  else
  {
    node[1] = a;
    node[0] = b;
  }
}

Node* Edge::getNeighbor(Node *n)
{
  if (node[0] == n)
    return node[1];
  else
    return node[0];
}

Graph::Graph(const std::vector<edge>& e)
{
  for (int i = 0; i < e.size(); i++)
  {
    addEdge(e[i].group_a, e[i].group_b, e[i].dist, 0);
  }

//  for(auto i = nodes_map_.begin(); i != nodes_map_.end(); i++)
//  {
//    std::cout << "node" << i->first << " neighbor ";
//    for (int j = 0; j < i->second->edges.size(); j++)
//    {
//      std::cout << i->second->edges[j]->getNeighbor(i->second)->id << " ";
//    }
//    std::cout << std::endl;
//  }

//  std::priority_queue<Node*, std::vector<Node*>, CompareWeight> node_queue;
//  for (auto i = nodes_map_.begin(); i != nodes_map_.end(); i++)
//  {
//    i->second->key = (int)(i->first)*10;
//    node_queue.push(i->second);
//  }
//  while (!node_queue.empty())
//  {
//    Node* n = node_queue.top();
//    node_queue.pop();
//    std::cout << n->id << " " << n->key << std::endl;
//  }
}

Graph::~Graph()
{

}

void Graph::test()
{
}

void Graph::addEdge(const int& v1, const int& v2, const int& w, const int& i)
{
//  std::cout << v1 << " " << v2 << " " << w << std::endl;
  Node* node_a;
  if (nodes_map_.find(v1) != nodes_map_.end())
  {
    node_a = (*nodes_map_.find(v1)).second;
  }
  else
  {
    node_a = new Node(v1);
    nodes_map_[v1] = node_a;
    nodes.push_back(node_a);
  }

  Node* node_b;
  if (nodes_map_.find(v2) != nodes_map_.end())
  {
    node_b = (*nodes_map_.find(v2)).second;
  }
  else
  {
    node_b = new Node(v2);
    nodes_map_[v2] = node_b;
    nodes.push_back(node_b);
  }

  Edge* g_e;
  g_e = new Edge(node_a, node_b, w, i);
  edges.push_back(g_e);
  node_a->edges.push_back(g_e);
  node_b->edges.push_back(g_e);
}

bool Graph::solveMST()
{
  std::priority_queue<Node*, std::vector<Node*>, CompareWeight> node_queue;
  for (int i = 0; i < nodes.size(); i++)
  {
    nodes[i]->key = std::numeric_limits<int>::max();
//    nodes[i]->visited = false;
//    nodes[i]->prev = nullptr;
    node_queue.push(nodes[i]);
  }

  node_queue.top()->key = 0;
  node_queue.top()->visited = true;
  while (!node_queue.empty())
  {
    Node* n_current = node_queue.top();
    node_queue.pop();

    for (int i = 0; i < n_current->edges.size(); i++)
    {
      Node* n_adj = n_current->edges[i]->getNeighbor(n_current);
      if (n_current->edges[i]->weight < n_adj->key && n_adj->visited == false)
      {
        n_adj->prev = n_current;
        n_adj->key = n_current->edges[i]->weight;
        n_adj->visited = true;
//        std::cout << n_adj->id << " ";
      }
    }
//    std::cout << std::endl;
  }
}


//bool Graph::solveAStar(point start, point goal)
//{
//  Node* n_start = new Node(start);
//  Node* n_goal = new Node(goal);
//  std::priority_queue<Node*> open_pq;
//  std::set<Node*> open_set, closed_set;

//  n_start->G = 0;
//  n_start->H = getDist(n_start->getLoc(), n_goal->getLoc());
//  n_start->F = n_start->G + n_start->H;

//  open_pq.push(n_start);
//  open_set.insert(n_start);
//  while(!open_pq.empty())
//  {
//    Node* n_current = open_pq.top();
//    open_pq.pop();
//    closed_set.insert(n_current);

//    if (n_current->getLoc() == n_goal->getLoc())
//    {
//      // generate path
//      return true;
//    }

//    int g_candidate = 0;
//    point p_neighbor;
//    p_neighbor.x = n_current->getLoc().x;
//    p_neighbor.y = n_current->getLoc().y;
//    for (int i = 0; i < 4; i++)
//    {
//      if (i == 0)
//        p_neighbor.x++;
//      else if ( i == 1)
//        p_neighbor.x--;
//      else if (i == 2)
//        p_neighbor.y++;
//      else
//        p_neighbor.y--;

//      Node* n_neighbor = new Node(p_neighbor);
//      if (closed_set.find(n_neighbor) != closed_set.end())
//      {
//        // neigbor in close set
//        continue;
//      }
////      else if (open_pq)
////      {
////        // not in the open set
////      }

////      g_candidate = n_current.g
//    }
//  }

//  return false;
//}

std::map<int, Node*> Graph::getResult()
{
  return nodes_map_;
}

bool Graph::findNodeInSet(const Node &n, const std::set<Node>& pq)
{

}

int Graph::getDist(const point& lhs, const point& rhs)
{
  return std::abs(lhs.x-rhs.x)+std::abs(lhs.y-rhs.y);
}
