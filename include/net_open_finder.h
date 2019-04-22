#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include <graph.h>

enum location { LL, UR, LR, UL};

struct routed_shape
{
  point ur_pt;
  point ll_pt;

  int layer;
  int group = -1;
  int index;

  int ll_index = -1;
  int ul_index = -1;
  int lr_index = -1;
  int ur_index = -1;

  int x_shape_index = -1;
  int y_shape_index = -1;
};

struct routed_via
{
  int layer;
  point position;
  bool connected = false;
};

struct obstacle
{
  point ur_pt;
  point ll_pt;
  int layer;
  int index;
  int group = -1;

  int ll_index = -1;
  int ul_index = -1;
  int lr_index = -1;
  int ur_index = -1;

  int x_shape_index = -1;
  int y_shape_index = -1;
};

struct rs_group
{
  int id = -1;
  std::vector<routed_shape> member;
  std::vector<edge> edges;

  int max_x;
  int max_y;
  int min_x;
  int min_y;
};

struct ob_group
{
  int id = -1;
  std::vector<obstacle> member;
//  std::vector<edge> edges;

  int max_x;
  int max_y;
  int min_x;
  int min_y;
};

struct shape
{
  point ur_pt;
  point ll_pt;

  int layer;
  int group = -1;
  int index;
  int type; // 0 rs, 1 ob
};

class NetOpenFinder
{
public:

  NetOpenFinder(const int& via_cost,
                const int& spacing,
                const int& number_layer,
                const point boundary,
                std::vector<routed_shape> routed_shapes,
                std::vector<routed_via> routed_vias,
                std::vector<obstacle> obstacles);

  ~NetOpenFinder();

  std::vector<line> solve();

  std::vector< std::vector<edge>> solve_2();

  std::vector<routed_shape> groupedShpaes(int group_id);


  void connectVias();

  std::vector< std::vector<routed_shape>> getRoutedShapes()
  {
    return sort_layer_rs_;
  }

  std::vector< std::vector<obstacle>> getObstacle()
  {
    return sort_layer_ob_;
  }

  std::vector<int> getGroupIndex()
  {
    return grouped_index_;
  }

  std::vector<edge> edges_;
  std::vector<edge> via_edge_;
  std::vector<routed_via> vias_;

private:

  std::vector<line> solveMST();

  std::vector< std::vector<edge>> solveMST_2();

  void initialize();

  void connectGroup();


  void findGroupShpaeCorner();

  void findGroupObstacleCorner();

  void removeSelfRedundantEdges();

  void removeGroupRedundantEdges();

  void groupTouchedShapes(std::vector<std::vector<routed_shape>>& rs);

  void groupTouchedObstacle(std::vector<std::vector<obstacle> >& rs);

  void updateGroupIndex(routed_shape& primary_shape, routed_shape& append_shape);

  void updateGroupIndex(obstacle& primary_shape, obstacle& append_shape);

  void addShapeToGroup(const int& group_index, routed_shape& rs);

  void addShapeToGroup(const int& group_index, obstacle& rs);

  void connectClosestShape(int i, int j, int current_index, location loc);

  edge getClosestEdge(const routed_shape& current_rs, const routed_shape& neighbor_rs, const location loc);

  void generateCollisionFreePath();

  bool isCollision(edge e, std::vector<int>& obs);

  bool isCollision(int rs_a_index, int rs_b_index, point edge_pt_a, point edge_pt_b, obstacle ob, point& ob_intersect_pt);

  bool isCollision(edge e);

  bool isCollision_2(edge e);

  bool isInsideShape(const point& pt, int& closest_shape, int& type);

  std::vector<int> searchObstacle(const edge& e);

//  std::vector<line> findPath(const edge& e, std::vector<int> obs);
  edge findPath(routed_shape shapeA, routed_shape shapeB, point pointA, point pointB);

  // find two shape have touch or not using the corner information
  bool isTouched(const routed_shape& rs_a, const routed_shape& rs_b);

  bool isTouched_2(const routed_shape& rs_a, const routed_shape& rs_b);

  bool isTouched(const obstacle& ob_a, const obstacle& ob_b);

  int distToUR(const routed_shape& rs);

  int distToUL(const routed_shape& rs);

  int distToLR(const routed_shape& rs);

  int distToUR(const obstacle& ob);

  int distToUL(const obstacle& ob);

  int distToLR(const obstacle& ob);

  point closestPoint(point x, obstacle y);

  int getDist(const point& a, const point& b);

  int computeEdgeDistance(edge e);

  void sortElementToLayer();

  Graph* graph_;

  edge*** edge_table_;

  std::vector< std::vector<shape>> sort_x_shapes_;
  std::vector< std::vector<shape>> sort_y_shapes_;

  std::vector< std::vector<routed_shape>> sort_ll_rs_;
  std::vector< std::vector<routed_shape>> sort_ul_rs_;
  std::vector< std::vector<routed_shape>> sort_lr_rs_;
  std::vector< std::vector<routed_shape>> sort_ur_rs_;

  std::vector< std::vector<obstacle>> sort_ll_ob_;
  std::vector< std::vector<obstacle>> sort_ul_ob_;
  std::vector< std::vector<obstacle>> sort_lr_ob_;
  std::vector< std::vector<obstacle>> sort_ur_ob_;

  std::vector< std::vector<routed_shape>> sort_layer_rs_;

  std::vector< std::vector<routed_via>> sort_layer_rv_;

  std::vector< std::vector<obstacle>> sort_x_ob_;
  std::vector< std::vector<obstacle>> sort_y_ob_;

  std::vector< std::vector<obstacle>> sort_layer_ob_;

  std::vector< std::vector<int>> all_x;

  std::vector< std::vector<rs_group>> rs_groups_;
  std::vector< std::vector<ob_group>> ob_groups_;

  int num_group_;

  std::vector<int> grouped_index_;

  std::vector<routed_shape> raw_rs_;
  std::vector<routed_via> raw_vias_;
  std::vector<obstacle> raw_ob_;

  point upper_bound_;

  int via_cost_;
  int spaceing_;
  int num_layer_;
};
