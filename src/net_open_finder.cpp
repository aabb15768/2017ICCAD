#include "net_open_finder.h"

NetOpenFinder::NetOpenFinder(const int& via_cost,
                             const int& spacing,
                             const int& number_layer,
                             const point boundary,
                             std::vector<routed_shape> routed_shapes,
                             std::vector<routed_via> routed_vias,
                             std::vector<obstacle> obstacles):
  via_cost_(via_cost),
  spaceing_(spacing),
  num_layer_(number_layer),
  upper_bound_(boundary),
  raw_rs_(routed_shapes),
  raw_vias_(routed_vias),
  raw_ob_(obstacles)
{
  // initialize private parameter
  initialize();

  // rearrange the index of each layer to start at 0
  for (int i = 1; i < sort_layer_rs_.size(); i++)
  {
    int prev_cnt = 0;
    for (int j = 0; j < i; j++)
      prev_cnt += sort_layer_rs_[j].size();

    for (int j = 0; j < sort_layer_rs_[i].size(); j++)
      sort_layer_rs_[i][j].index -= prev_cnt;
  }

  for (int i = 1; i < sort_layer_ob_.size(); i++)
  {
    int prev_cnt = 0;
    for (int j = 0; j < i; j++)
      prev_cnt += sort_layer_ob_[j].size();

    for (int j = 0; j < sort_layer_ob_[i].size(); j++)
      sort_layer_ob_[i][j].index -= prev_cnt;
  }

  std::cout << "intialize complete" << std::endl;
  // sort routed shapes according to its position
  std::chrono::duration<double> total_time;
  auto start_time = std::chrono::high_resolution_clock::now();
  auto initial_time = std::chrono::high_resolution_clock::now();
  sort_ll_rs_ = sort_layer_rs_;
  for (int i = 0; i < sort_ll_rs_.size(); i++)
  {
    std::sort(sort_ll_rs_[i].begin(), sort_ll_rs_[i].end(),
              [] (const routed_shape& a, const routed_shape& b) { return ((a.ll_pt.x+a.ll_pt.y) < (b.ll_pt.x+b.ll_pt.y)); });
  }

  sort_ul_rs_ = sort_layer_rs_;
  for (int i = 0; i < sort_ul_rs_.size(); i++)
  {
    std::sort(sort_ul_rs_[i].begin(), sort_ul_rs_[i].end(),
              [this] (const routed_shape& a, const routed_shape& b) { return (distToUL(a) < distToUL(b)); });
  }

  sort_lr_rs_ = sort_layer_rs_;
  for (int i = 0; i < sort_lr_rs_.size(); i++)
  {
    std::sort(sort_lr_rs_[i].begin(), sort_lr_rs_[i].end(),
              [this] (const routed_shape& a, const routed_shape& b) { return (distToLR(a) < distToLR(b)); });
  }

  sort_ur_rs_ = sort_layer_rs_;
  for (int i = 0; i < sort_ur_rs_.size(); i++)
  {
    std::sort(sort_ur_rs_[i].begin(), sort_ur_rs_[i].end(),
              [this] (const routed_shape& a, const routed_shape& b) { return (distToUR(a) < distToUR(b)); });
  }

  for (int i = 0; i < sort_layer_rs_.size(); i++)
  {
    for (int j = 0; j < sort_layer_rs_[i].size(); j++)
    {
      sort_layer_rs_[i][sort_ll_rs_[i][j].index].ll_index = j;
      sort_layer_rs_[i][sort_ul_rs_[i][j].index].ul_index = j;
      sort_layer_rs_[i][sort_lr_rs_[i][j].index].lr_index = j;
      sort_layer_rs_[i][sort_ur_rs_[i][j].index].ur_index = j;
    }
  }

  sort_ll_ob_ = sort_layer_ob_;
  for (int i = 0; i < sort_ll_ob_.size(); i++)
  {
    std::sort(sort_ll_ob_[i].begin(), sort_ll_ob_[i].end(),
              [] (const obstacle& a, const obstacle& b) { return ((a.ll_pt.x+a.ll_pt.y) < (b.ll_pt.x+b.ll_pt.y)); });
  }

  sort_ul_ob_ = sort_layer_ob_;
  for (int i = 0; i < sort_ul_ob_.size(); i++)
  {
    std::sort(sort_ul_ob_[i].begin(), sort_ul_ob_[i].end(),
              [this] (const obstacle& a, const obstacle& b) { return (distToUL(a) < distToUL(b)); });
  }

  sort_lr_ob_ = sort_layer_ob_;
  for (int i = 0; i < sort_lr_ob_.size(); i++)
  {
    std::sort(sort_lr_ob_[i].begin(), sort_lr_ob_[i].end(),
              [this] (const obstacle& a, const obstacle& b) { return (distToLR(a) < distToLR(b)); });
  }

  sort_ur_ob_ = sort_layer_ob_;
  for (int i = 0; i < sort_ur_ob_.size(); i++)
  {
    std::sort(sort_ur_ob_[i].begin(), sort_ur_ob_[i].end(),
              [this] (const obstacle& a, const obstacle& b) { return (distToUR(a) < distToUR(b)); });
  }

  for (int i = 0; i < sort_layer_ob_.size(); i++)
  {
    for (int j = 0; j < sort_layer_ob_[i].size(); j++)
    {
      sort_layer_ob_[i][sort_ll_ob_[i][j].index].ll_index = j;
      sort_layer_ob_[i][sort_ul_ob_[i][j].index].ul_index = j;
      sort_layer_ob_[i][sort_lr_ob_[i][j].index].lr_index = j;
      sort_layer_ob_[i][sort_ur_ob_[i][j].index].ur_index = j;
    }
  }

  sort_x_ob_ = sort_layer_ob_;
  for (int i = 0; i < sort_x_ob_.size(); i++)
  {
    std::sort(sort_x_ob_[i].begin(), sort_x_ob_[i].end(),
              [this] (const obstacle& a, const obstacle& b) { return (a.ll_pt.x < b.ll_pt.x); });
  }

  sort_y_ob_ = sort_layer_ob_;
  for (int i = 0; i < sort_y_ob_.size(); i++)
  {
    std::sort(sort_y_ob_[i].begin(), sort_y_ob_[i].end(),
              [this] (const obstacle& a, const obstacle& b) { return (a.ll_pt.y < b.ll_pt.y); });
  }

  std::cout << "sort routed shapes complete" << std::endl;
  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = end_time - start_time;
  total_time += elapsed_time;
  std::cout << "take " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() << " ms to sort all shapes.\n";

  start_time = std::chrono::high_resolution_clock::now();

  groupTouchedShapes(sort_layer_rs_);

  end_time = std::chrono::high_resolution_clock::now();
  elapsed_time = end_time - start_time;
  std::cout << "take " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() << " ms to group the shape.\n";

  // remove the group which does not contain member
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    auto rm_itr = std::remove_if(rs_groups_[i].begin(), rs_groups_[i].end(),
                          [](const rs_group& rs){ return rs.member.size() == 0;});
    rs_groups_[i].erase(rm_itr, rs_groups_[i].end());
  }

  for (int i = 0; i < rs_groups_.size(); i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      rs_groups_[i][j].id = j;
      for (int k = 0; k < rs_groups_[i][j].member.size(); k++)
      {
        rs_groups_[i][j].member[k].group = j;
        sort_layer_rs_[i][rs_groups_[i][j].member[k].index].group = j;
        sort_ll_rs_[i][rs_groups_[i][j].member[k].ll_index].group = j;
        sort_lr_rs_[i][rs_groups_[i][j].member[k].lr_index].group = j;
        sort_ul_rs_[i][rs_groups_[i][j].member[k].ul_index].group = j;
        sort_ur_rs_[i][rs_groups_[i][j].member[k].ur_index].group = j;
      }
    }
  }

  edge_table_ = new edge**[rs_groups_.size()];
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    edge_table_[i] = new edge*[rs_groups_[i].size()];
  }
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      edge_table_[i][j] = new edge[j+1];
      for (int k = 0; k < j+1; k++)
      {
        edge_table_[i][j][k].dist = -1;
      }
    }
  }

  // sort all the shape order in x and y
  for (int i = 0; i < sort_layer_rs_.size(); i++)
  {
    for (int j = 0; j < sort_layer_rs_[i].size(); j++)
    {
      shape s;
      s.index = sort_layer_rs_[i][j].index;
      s.type = 0;
      s.layer = sort_layer_rs_[i][j].layer;
      s.group = sort_layer_rs_[i][j].group;
      s.ur_pt = sort_layer_rs_[i][j].ur_pt;
      s.ll_pt = sort_layer_rs_[i][j].ll_pt;
      sort_x_shapes_[i].push_back(s);
      sort_y_shapes_[i].push_back(s);
    }

    for (int j = 0; j < sort_x_ob_[i].size(); j++)
    {
      shape s;
      s.index = sort_x_ob_[i][j].index;
      s.type = 1;
      s.layer = sort_layer_rs_[i][j].layer;
      s.ur_pt = sort_x_ob_[i][j].ur_pt;
      s.ll_pt = sort_x_ob_[i][j].ll_pt;
      sort_x_shapes_[i].push_back(s);
      sort_y_shapes_[i].push_back(s);
    }

    std::sort(sort_x_shapes_[i].begin(), sort_x_shapes_[i].end(),
              [this](const shape& a, const shape& b) { return (a.ll_pt.x < b.ll_pt.x); });



    std::sort(sort_y_shapes_[i].begin(), sort_y_shapes_[i].end(),
              [this] (const shape& a, const shape& b) { return (a.ll_pt.y < b.ll_pt.y); });

    for (int j = 0; j < sort_x_shapes_[i].size(); j++)
    {
      if (sort_x_shapes_[i][j].type == 0)
      {
        sort_layer_rs_[i][sort_x_shapes_[i][j].index].x_shape_index = j;
      }
      else
      {
        sort_layer_ob_[i][sort_x_shapes_[i][j].index].x_shape_index = j;
      }
    }

    for (int j = 0; j < sort_y_shapes_[i].size(); j++)
    {
      if (sort_y_shapes_[i][j].type == 0)
      {
        sort_layer_rs_[i][sort_y_shapes_[i][j].index].y_shape_index = j;
      }
      else
      {
        sort_layer_ob_[i][sort_y_shapes_[i][j].index].y_shape_index = j;
      }
    }
  }


//  groupTouchedObstacle(sort_layer_ob_);

  for (int i = 0; i < ob_groups_.size(); i++)
  {
    auto rm_itr = std::remove_if(ob_groups_[i].begin(), ob_groups_[i].end(),
                          [](const ob_group& ob){ return ob.member.size() == 0;});
    ob_groups_[i].erase(rm_itr, ob_groups_[i].end());
  }

//  findGroupObstacleCorner();

  // find max xy and min xy of each group
  start_time = std::chrono::high_resolution_clock::now();
  findGroupShpaeCorner();

  // connect groups with corner and closet shape to the corner
  connectGroup();

  end_time = std::chrono::high_resolution_clock::now();
  elapsed_time = end_time - start_time;
  total_time += elapsed_time;
  std::cout << "take " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
               " ms to find group corner and connect group.\n";

  // sort edge in order of dist
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
        std::sort(rs_groups_[i][j].edges.begin(), rs_groups_[i][j].edges.end(),
                  [this] (const edge& a, const edge& b) { return a.dist < b.dist;} );
    }
  }

//  removeSelfRedundantEdges();
//  removeGroupRedundantEdges();

//  for (int i = 0; i < rs_groups_.size(); i++)
//  {
//    for (int j = 0; j < rs_groups_[i].size(); j++)
//    {
//      for (int k = 0; k < rs_groups_[i][j].edges.size(); k++)
//      {
//        if (rs_groups_[i][j].edges[k].dist > 0)
//          edges_.push_back(rs_groups_[i][j].edges[k]);
//      }
//    }
//  }

//  int cnt = 0;
//  for (int i = 0; i < rs_groups_.size(); i++)
//  {
//    for (int j = 0; j < rs_groups_[i].size(); j++)
//    {
//      for (int k = 0; k < j; k++)
//      {
//        if (j == k)
//        {
//          if (edge_table_[i][j][k].dist != -1)
//            std::cout << "wrong1" << std::endl;
//          continue;
//        }
//        if (edge_table_[i][j][k].dist >=0)
//        {
//          edges_.push_back(edge_table_[i][j][k]);
//          cnt++;
//        }
//        else if(edge_table_[i][j][k].dist < -1)
//          std::cout << "wrong" << std::endl;
//      }
//    }
//  }

//  std::cout << "total edges:" << edges_.size() << std::endl;

  end_time = std::chrono::high_resolution_clock::now();
  elapsed_time = end_time - initial_time;
  std::cout << "take " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
               " ms to finish the pre-processing.\n";
}

NetOpenFinder::~NetOpenFinder() {}

std::vector<line> NetOpenFinder::solve()
//std::vector< std::vector<edge>> NetOpenFinder::solve()
{
  std::cout << "solving MST..." << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();

  auto mst_lines = solveMST();
//  auto mst_lines = solveMST_2();

  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = end_time - start_time;
  std::cout << "take " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
               " ms to solve MST.\n";

  return mst_lines;
}

std::vector< std::vector<edge>> NetOpenFinder::solve_2()
{
  std::cout << "solving MST..." << std::endl;
  auto start_time = std::chrono::high_resolution_clock::now();

//  auto mst_lines = solveMST();
  auto mst_lines = solveMST_2();

  auto end_time = std::chrono::high_resolution_clock::now();
  auto elapsed_time = end_time - start_time;
  std::cout << "take " << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count() <<
               " ms to solve MST.\n";

  return mst_lines;
}

std::vector<line> NetOpenFinder::solveMST()
{
  std::vector<line> lines;
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    std::vector<edge> all_edge;
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      for (int k = 0; k < j; k++)
      {
        if (j != k && edge_table_[i][j][k].dist != -1)
          all_edge.push_back(edge_table_[i][j][k]);
      }
    }

    std::unique_ptr<Graph> mst_solver(new Graph(all_edge));
    mst_solver->solveMST();

    std::map<int, Node*> n_map = mst_solver->getResult();
    int cnt = 0;
    for (auto j = n_map.begin(); j != n_map.end(); j++)
    {
      if (j->second->prev != nullptr)
      {
        edge e;
        if (j->first > j->second->prev->id)
          e = edge_table_[i][j->first][j->second->prev->id];
        else
          e = edge_table_[i][j->second->prev->id][j->first];
//        edge e = rs_groups_[i][j->second->prev->id].edges[j->second->prev_i];

//        std::cout << (int)(j->first) << " " << (int)(j->second->prev->id) << " " << (int)(j->second->key) << std::endl;
        edges_.push_back(e);
      }
      else
      {
//        std::cout << (int)(j->first) << std::endl;
        cnt++;
      }
    }
  }

  return lines;
}

std::vector< std::vector<edge>> NetOpenFinder::solveMST_2()
{
  edge*** edge_table_2;
  edge_table_2 = new edge**[rs_groups_.size()];
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    edge_table_2[i] = new edge*[rs_groups_[i].size()];
  }
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      edge_table_2[i][j] = new edge[j+1];
      for (int k = 0; k < j+1; k++)
      {
        edge_table_2[i][j][k].dist = -1;
      }
    }
  }

  std::vector< std::vector<edge>> mst;
  std::vector< std::vector<int>> mst_i;
  mst.resize(rs_groups_.size());
  mst_i.resize(rs_groups_.size());

  for (int i = 0; i < rs_groups_.size(); i++)
  {
    mst_i[i].push_back(rs_groups_.size()-1);

    for (int j = 0; j < rs_groups_[i].size()-1; j++)
    {
//      std::cout << "1" << std::endl;
        for (int k = 0; k < mst_i[i][j]; k++)
        {
            if(edge_table_[i][mst_i[i][j]][k].dist != -1) {
                edge_table_2[i][mst_i[i][j]][k] = edge_table_[i][mst_i[i][j]][k];
                edge_table_[i][mst_i[i][j]][k].dist = -1;
            }
        }

//        std::cout << "2" << std::endl;
        for (int k = mst_i[i][j]+1; k < rs_groups_[i].size(); k++)
        {
            if(edge_table_[i][k][mst_i[i][j]].dist != -1) {
                edge_table_2[i][k][mst_i[i][j]] = edge_table_[i][k][mst_i[i][j]];
                edge_table_[i][k][mst_i[i][j]].dist = -1;
            }

        }

//        std::cout << "3" << std::endl;
        for(int k = 0; k < mst_i[i].size(); k++) {
            for(int m = 0; m < mst_i[i].size(); m++) {
                if(m == k) {
                    continue;
                }
                int max;
                int min;
                if(mst_i[i][k] < mst_i[i][m]) {
                    min = mst_i[i][k];
                    max = mst_i[i][m];
                } else if (mst_i[i][k] > mst_i[i][m]){
                    min = mst_i[i][m];
                    max = mst_i[i][k];
                }
                else
                {
                  std::cout << "wrong " << j << std::endl;
                }
                edge_table_2[i][max][min].dist = -1;
            }
        }

        int min_dist = std::numeric_limits<int>::max();
        int max_group;
        int min_group;

//        std::cout << "4" << std::endl;
        for(int k = 0; k < rs_groups_[i].size(); k++) {
            for(int m = 0; m < k; m++) {
                if(m == k) {
                    continue;
                }
                if(edge_table_2[i][k][m].dist == -1) {
                    continue;
                } else {
                    if(edge_table_2[i][k][m].dist < min_dist) {
                        min_dist = edge_table_2[i][k][m].dist;
                        max_group = k;
                        min_group = m;
                    }
                }

            }
        }

        mst[i].push_back(edge_table_2[i][max_group][min_group]);
        edge_table_2[i][max_group][min_group].dist = -1;

        int debug = 0;
        bool ctr = true;

//        std::cout << "5" << std::endl;
        for(int k = 0; k < mst_i[i].size(); k++) {
//          std::cout << min_group << std::endl;
            if(min_group == mst_i[i][k]) {
                ctr = false;
            }
        }


        if(ctr) {
            mst_i[i].push_back(min_group);
            debug = 1;
        }

        ctr = !ctr;
        for(int k = 0; k < mst_i[i].size(); k++) {
//          std::cout << min_group << std::endl;
            if(max_group == mst_i[i][k]) {
                ctr = false;
            }
        }

        if (ctr) {
            mst_i[i].push_back(max_group);
            debug = 2;
        }

        if (debug == 0)
        {
//          std::cout << "debug" << std::endl;
        }

//        std::cout << "6" << std::endl;
    }
  }
  return mst;
}

void NetOpenFinder::initialize()
{
  sort_layer_rs_.resize(num_layer_);
  sort_layer_rv_.resize(num_layer_);
  sort_layer_ob_.resize(num_layer_);

  sort_x_shapes_.resize(num_layer_);
  sort_y_shapes_.resize(num_layer_);

  rs_groups_.resize(num_layer_);
  ob_groups_.resize(num_layer_);

  sortElementToLayer();

  for (int i = 0; i < rs_groups_.size(); i++)
  {
    rs_groups_[i].resize(sort_layer_rs_[i].size());
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      rs_groups_[i][j].id = j;
    }
  }

  for (int i = 0; i < ob_groups_.size(); i++)
  {
    ob_groups_[i].resize(sort_layer_ob_[i].size());
    for (int j = 0; j < ob_groups_[i].size(); j++)
    {
      ob_groups_[i][j].id = j;
    }
  }
}

void NetOpenFinder::connectGroup()
{
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      // max x, find closest UR neighbor
      int index = rs_groups_[i][j].max_x;
      connectClosestShape(i, j, index, LL);
      connectClosestShape(i, j, index, LR);
      connectClosestShape(i, j, index, UL);
      connectClosestShape(i, j, index, UR);

      index = rs_groups_[i][j].min_x;
      connectClosestShape(i, j, index, LL);
      connectClosestShape(i, j, index, LR);
      connectClosestShape(i, j, index, UL);
      connectClosestShape(i, j, index, UR);

      index = rs_groups_[i][j].max_y;
      connectClosestShape(i, j, index, LL);
      connectClosestShape(i, j, index, LR);
      connectClosestShape(i, j, index, UL);
      connectClosestShape(i, j, index, UR);

      index = rs_groups_[i][j].min_y;
      connectClosestShape(i, j, index, LL);
      connectClosestShape(i, j, index, LR);
      connectClosestShape(i, j, index, UL);
      connectClosestShape(i, j, index, UR);
    }
  }
}

void NetOpenFinder::connectVias()
{
  for (int i = 0; i < sort_layer_rv_.size()-1; i++)
  {
    bool is_finded = false;
    int is_count = 0;
//    for (int j = 0; j < sort_layer_rs_[i].size(); j++)
    for (int j = sort_layer_rs_[i].size()-1; j >= 0; j--)
    {
//      for (int k = 0; k < sort_layer_rs_[i+1].size(); k++)
      for (int k = sort_layer_rs_[i+1].size()-1; k >= 0; k--)
      {
        edge e;
        e.shape_a_pt = sort_layer_rs_[i][j].ll_pt;
        e.shape_b_pt = sort_layer_rs_[i+1][k].ur_pt;
        e.layer = sort_layer_rs_[i+1][k].layer;
        if (!isCollision(e))
        {
          via_edge_.push_back(e);

          routed_via v;
          v.position = sort_layer_rs_[i][j].ll_pt;
          v.layer = sort_layer_rs_[i][j].layer;
          vias_.push_back(v);
          is_finded = true;
          is_count++;
//          e.layer -= 1;
//          via_edge_.push_back(e);
        }

        if (is_count == 1)
          break;
      }
      if (is_count == 1)
        break;
    }
//    if (sort_layer_rv_[i].size() == 0)
//    {
      // define a via

//    }
//    else
//    {
      // connect to closest group
//      for (int j = 0; j < sort_layer_rv_[i].size(); j++)
//      {
//        // if inside shape then do nothing
//        for (int k = 0; k < sort_layer_rs_[i].size(); k++)
//        {
//          routed_shape rs_1, rs_2;
//          edge e;
//          e.shape_a_pt = sort_layer_rv_[i][j].position;
//          e.shape_b_pt = sort_layer_rs_[i][k].ll_pt;
//          e.layer = sort_layer_rs_[i][k].layer;
//          if (!isCollision(e))
//          {
//            edges_.push_back(e);
//            break;
//          }
//        }
//      }
//    }
  }
}

void NetOpenFinder::findGroupShpaeCorner()
{
  for (int layer = 0; layer < rs_groups_.size(); layer++)
  {
    for (int i = 0; i < rs_groups_[layer].size(); i++)
    {
      int max_x = 0, min_x = upper_bound_.x, max_y = 0, min_y = upper_bound_.y;
      for (int j = 0; j < rs_groups_[layer][i].member.size(); j++)
      {
        if (rs_groups_[layer][i].member[j].ur_pt.x > max_x)
        {
          max_x = rs_groups_[layer][i].member[j].ur_pt.x;
          rs_groups_[layer][i].max_x = rs_groups_[layer][i].member[j].index;
        }

        if (rs_groups_[layer][i].member[j].ur_pt.y > max_y)
        {
          max_y = rs_groups_[layer][i].member[j].ur_pt.y;
          rs_groups_[layer][i].max_y = rs_groups_[layer][i].member[j].index;
        }

        if (rs_groups_[layer][i].member[j].ll_pt.x < min_x)
        {
          min_x = rs_groups_[layer][i].member[j].ll_pt.x;
          rs_groups_[layer][i].min_x = rs_groups_[layer][i].member[j].index;
        }

        if (rs_groups_[layer][i].member[j].ll_pt.y < min_y)
        {
          min_y = rs_groups_[layer][i].member[j].ll_pt.y;
          rs_groups_[layer][i].min_y = rs_groups_[layer][i].member[j].index;
        }
      }
    }
  }
}

void NetOpenFinder::findGroupObstacleCorner()
{
  for (int layer = 0; layer < ob_groups_.size(); layer++)
  {
    for (int i = 0; i < ob_groups_[layer].size(); i++)
    {
      int max_x = 0, min_x = upper_bound_.x, max_y = 0, min_y = upper_bound_.y;
      for (int j = 0; j < ob_groups_[layer][i].member.size(); j++)
      {
        if (ob_groups_[layer][i].member[j].ur_pt.x > max_x)
        {
          max_x = ob_groups_[layer][i].member[j].ur_pt.x;
          ob_groups_[layer][i].max_x = ob_groups_[layer][i].member[j].ur_pt.x;
        }

        if (ob_groups_[layer][i].member[j].ur_pt.y > max_y)
        {
          max_y = ob_groups_[layer][i].member[j].ur_pt.y;
          ob_groups_[layer][i].max_y = ob_groups_[layer][i].member[j].ur_pt.y;
        }

        if (ob_groups_[layer][i].member[j].ll_pt.x < min_x)
        {
          min_x = ob_groups_[layer][i].member[j].ll_pt.x;
          ob_groups_[layer][i].min_x = ob_groups_[layer][i].member[j].ll_pt.x;
        }

        if (ob_groups_[layer][i].member[j].ll_pt.y < min_y)
        {
          min_y = ob_groups_[layer][i].member[j].ll_pt.y;
          ob_groups_[layer][i].min_y = ob_groups_[layer][i].member[j].ll_pt.y;
        }
      }
    }
  }
}

void NetOpenFinder::removeSelfRedundantEdges()
{
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      for (int k = 0; k < rs_groups_[i][j].edges.size(); k++)
      {
        std::vector<int> remove_list;
        for (int m = 0; m < rs_groups_[i][j].edges.size(); m++)
        {
          if (k == m)
            continue;

          int k_a = sort_layer_rs_[i][rs_groups_[i][j].edges[k].shape_a].group;
          int k_b = sort_layer_rs_[i][rs_groups_[i][j].edges[k].shape_b].group;
          int m_a = sort_layer_rs_[i][rs_groups_[i][j].edges[m].shape_a].group;
          int m_b = sort_layer_rs_[i][rs_groups_[i][j].edges[m].shape_b].group;

          if ((k_a == m_a && k_b == m_b) || (k_a == m_b && k_b == m_a))
          {
            if (rs_groups_[i][j].edges[k].dist <= rs_groups_[i][j].edges[m].dist)
            {
              remove_list.push_back(m);
            }
          }
        }

        for (int m = remove_list.size()-1; m >= 0; m--)
        {
          std::swap(rs_groups_[i][j].edges[remove_list[m]], rs_groups_[i][j].edges.back());
          rs_groups_[i][j].edges.pop_back();
        }
      }
    }
  }
}

void NetOpenFinder::removeGroupRedundantEdges()
{
  for (int i = 0; i < rs_groups_.size(); i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      for (int k = 0; k < rs_groups_[i].size(); k++)
      {
        if (j == k)
          continue;

        for (int m = 0; m < rs_groups_[i][j].edges.size(); m++)
        {
          for (int n = 0; n < rs_groups_[i][k].edges.size(); n++)
          {
            int n_a = sort_layer_rs_[i][rs_groups_[i][k].edges[n].shape_a].group;
            int m_a = sort_layer_rs_[i][rs_groups_[i][j].edges[m].shape_a].group;
            int n_b = sort_layer_rs_[i][rs_groups_[i][k].edges[n].shape_b].group;
            int m_b = sort_layer_rs_[i][rs_groups_[i][j].edges[m].shape_b].group;
            if ((n_a == m_a && n_b == m_b) || (n_a == m_b && n_b == m_a))
            {
              int index = 0;
              int group = 0;
              if (rs_groups_[i][k].edges[n].dist >= rs_groups_[i][j].edges[m].dist)
              {
                group = k;
                index = n;
              }
              else
              {
                group = j;
                index = m;
              }

              if (group == k)
              {
                rs_groups_[i][group].edges.erase(rs_groups_[i][group].edges.begin()+n);
                n--;
              }
              else
              {
                rs_groups_[i][group].edges.erase(rs_groups_[i][group].edges.begin()+m);
                m--;
              }
              break;
            }
          }
        }
      }
    }
  }
//  for (int i = 0; i < rs_groups_.size(); i++)
//  {
//    int edge_cnt = 0;
//    for (int j = 0; j < rs_groups_[i].size(); j++)
//    {
//      edge_cnt += rs_groups_[i][j].member.size();
//    }

//    std::vector< std::vector<int>> table(edge_cnt, std::vector<int>(edge_cnt, -1));
//    for (int j = 0; j < rs_groups_[i].size(); j++)
//    {
//      for (int k = 0; k < rs_groups_[i][j].edges.size(); k++)
//      {
//        int lhs = rs_groups_[i][j].edges[k].group_a;
//        int rhs = rs_groups_[i][j].edges[k].group_b;
//        // if not in table, ++
//        // else check length and save the smaller one
//        if (table[lhs][rhs] == -1)
//        {

//        }
//        else
//        {

//        }
//      }
//    }
//  }

}

void NetOpenFinder::groupTouchedShapes(std::vector< std::vector<routed_shape>>& rs)
{
  for (int i = 0; i < rs.size(); i++)
  {
    for (int j = 0; j < rs[i].size(); j++)
    {
      int current = j;
      int current_index = sort_ll_rs_[i][current].index;
      if (rs[i][current_index].group == -1)
      {
        rs_groups_[i][j].member.push_back(sort_ll_rs_[i][current]);
        rs[i][current_index].group = rs_groups_[i][j].id;
      }

      int ur_neighbor = current+1;
      int neighbor_index = 0;
      if (ur_neighbor < rs[i].size())
      {
        neighbor_index = sort_ll_rs_[i][ur_neighbor].index;
        while (ur_neighbor != rs[i].size() &&
               (rs[i][neighbor_index].ll_pt.x <= rs[i][current_index].ll_pt.x ||
                rs[i][neighbor_index].ll_pt.y < rs[i][current_index].ll_pt.y))
        {
          ur_neighbor++;
          neighbor_index = sort_ll_rs_[i][ur_neighbor].index;
        }

        if ((ur_neighbor < rs[i].size()) && (rs[i][neighbor_index].group != rs[i][current_index].group))
        {
          if (isTouched(rs[i][neighbor_index], rs[i][current_index]))
          {
            if (rs[i][neighbor_index].group == -1)
            {
              addShapeToGroup(rs[i][current_index].group, rs[i][neighbor_index]);
            }
            else
            {
              updateGroupIndex(rs[i][current_index], rs[i][neighbor_index]);
            }
          }
        }
      }
//      current = rs[i][current_index].lr_index;
      current = rs[i][sort_ll_rs_[i][current].index].lr_index;
      int ul_neighbor = current + 1;
      if (ul_neighbor < rs[i].size())
      {
        current_index = sort_lr_rs_[i][current].index;
        neighbor_index = sort_lr_rs_[i][ul_neighbor].index;
        while (ul_neighbor != rs[i].size() &&
               ((upper_bound_.x-rs[i][neighbor_index].ll_pt.x) < (upper_bound_.x-rs[i][current_index].ll_pt.x) ||
                rs[i][neighbor_index].ll_pt.y <= rs[i][current_index].ll_pt.y))
        {
          ul_neighbor++;
          neighbor_index = sort_lr_rs_[i][ul_neighbor].index;
        }

        if ((ul_neighbor < rs[i].size()) && (rs[i][neighbor_index].group != rs[i][current_index].group))
        {
          if (isTouched(rs[i][neighbor_index], rs[i][current_index]))
          {
            if (rs[i][neighbor_index].group == -1)
            {
              addShapeToGroup(rs[i][current_index].group, rs[i][neighbor_index]);
            }
            else
            {
              updateGroupIndex(rs[i][current_index], rs[i][neighbor_index]);
            }
          }
        }
      }

      current = rs[i][sort_lr_rs_[i][current].index].ur_index;
      int ll_neighbor = current + 1;
      if (ll_neighbor < rs[i].size())
      {
        current_index = sort_ur_rs_[i][current].index;
        neighbor_index = sort_ur_rs_[i][ll_neighbor].index;
        while ((ll_neighbor != rs[i].size()) &&
               ((upper_bound_.x-rs[i][neighbor_index].ll_pt.x) <= (upper_bound_.x-rs[i][current_index].ll_pt.x) ||
                (upper_bound_.y-rs[i][neighbor_index].ll_pt.y) < (upper_bound_.y-rs[i][current_index].ll_pt.y)))
        {
          ll_neighbor++;
          neighbor_index = sort_ur_rs_[i][ll_neighbor].index;
        }

        if ((ll_neighbor < rs[i].size()) && (rs[i][neighbor_index].group != rs[i][current_index].group))
        {
          if (isTouched(rs[i][neighbor_index], rs[i][current_index]))
          {
            if (rs[i][neighbor_index].group == -1)
            {
              addShapeToGroup(rs[i][current_index].group, rs[i][neighbor_index]);
            }
            else
            {
              updateGroupIndex(rs[i][current_index], rs[i][neighbor_index]);
            }
          }
        }
      }

      current = rs[i][sort_ur_rs_[i][current].index].ul_index;
      int lr_neighbor = current + 1;
      if (lr_neighbor < rs[i].size())
      {
        current_index = sort_ul_rs_[i][current].index;
        neighbor_index = sort_ul_rs_[i][lr_neighbor].index;
        while ((lr_neighbor != rs[i].size()) &&
               ((rs[i][neighbor_index].ll_pt.x < rs[i][current_index].ll_pt.x) ||
                (upper_bound_.y-rs[i][neighbor_index].ll_pt.y) <= (upper_bound_.y-rs[i][current_index].ll_pt.y)))
        {
          lr_neighbor++;
          neighbor_index = sort_ul_rs_[i][lr_neighbor].index;
        }

        if ((lr_neighbor < rs[i].size()) && (rs[i][neighbor_index].group != rs[i][current_index].group))
        {
          if (isTouched(rs[i][neighbor_index], rs[i][current_index]))
          {
            if (rs[i][neighbor_index].group == -1)
            {
              addShapeToGroup(rs[i][current_index].group, rs[i][neighbor_index]);
            }
            else
            {
              updateGroupIndex(rs[i][current_index], rs[i][neighbor_index]);
            }
          }
        }
      }
    } // end shapes
  } // end each layer
}

void NetOpenFinder::groupTouchedObstacle(std::vector< std::vector<obstacle>>& ob)
{
  for (int i = 0; i < ob.size(); i++)
  {
    for (int j = 0; j < ob[i].size(); j++)
    {
      int current = j;
      int current_index = sort_ll_ob_[i][current].index;
      if (ob[i][current_index].group == -1)
      {
        ob_groups_[i][j].member.push_back(sort_ll_ob_[i][current]);
        ob[i][current_index].group = ob_groups_[i][j].id;
      }

      int ur_neighbor = current+1;
      int neighbor_index = 0;
      if (ur_neighbor < ob[i].size())
      {
        neighbor_index = sort_ll_ob_[i][ur_neighbor].index;
        while (ur_neighbor != ob[i].size() &&
               (ob[i][neighbor_index].ll_pt.x <= ob[i][current_index].ll_pt.x ||
                ob[i][neighbor_index].ll_pt.y < ob[i][current_index].ll_pt.y))
        {
          ur_neighbor++;
          neighbor_index = sort_ll_ob_[i][ur_neighbor].index;
        }

        if ((ur_neighbor < ob[i].size()) && (ob[i][neighbor_index].group != ob[i][current_index].group))
        {
          if (isTouched(ob[i][neighbor_index], ob[i][current_index]))
          {
            if (ob[i][neighbor_index].group == -1)
            {
              addShapeToGroup(ob[i][current_index].group, ob[i][neighbor_index]);


              ob_groups_[ob[i][current_index].layer-1][ob[i][current_index].group].member.push_back(ob[i][neighbor_index]);
              ob[i][neighbor_index].group = ob[i][current_index].group;
            }
            else
            {
              updateGroupIndex(ob[i][current_index], ob[i][neighbor_index]);
            }
          }
        }
      }
//      current = rs[i][current_index].lr_index;
      current = ob[i][sort_ll_ob_[i][current].index].lr_index;
      int ul_neighbor = current + 1;
      if (ul_neighbor < ob[i].size())
      {
        current_index = sort_lr_ob_[i][current].index;
        neighbor_index = sort_lr_ob_[i][ul_neighbor].index;
        while (ul_neighbor != ob[i].size() &&
               ((upper_bound_.x-ob[i][neighbor_index].ll_pt.x) < (upper_bound_.x-ob[i][current_index].ll_pt.x) ||
                ob[i][neighbor_index].ll_pt.y <= ob[i][current_index].ll_pt.y))
        {
          ul_neighbor++;
          neighbor_index = sort_lr_ob_[i][ul_neighbor].index;
        }

        if ((ul_neighbor < ob[i].size()) && (ob[i][neighbor_index].group != ob[i][current_index].group))
        {
          if (isTouched(ob[i][neighbor_index], ob[i][current_index]))
          {
            if (ob[i][neighbor_index].group == -1)
            {
              addShapeToGroup(ob[i][current_index].group, ob[i][neighbor_index]);
            }
            else
            {
              updateGroupIndex(ob[i][current_index], ob[i][neighbor_index]);
            }
          }
        }
      }

      current = ob[i][sort_lr_ob_[i][current].index].ur_index;
      int ll_neighbor = current + 1;
      if (ll_neighbor < ob[i].size())
      {
        current_index = sort_lr_ob_[i][current].index;
        neighbor_index = sort_lr_ob_[i][ll_neighbor].index;
        while ((ll_neighbor != ob[i].size()) &&
               ((upper_bound_.x-ob[i][neighbor_index].ll_pt.x) <= (upper_bound_.x-ob[i][current_index].ll_pt.x) ||
                (upper_bound_.y-ob[i][neighbor_index].ll_pt.y) < (upper_bound_.y-ob[i][current_index].ll_pt.y)))
        {
          ll_neighbor++;
          neighbor_index = sort_ur_ob_[i][ll_neighbor].index;
        }

        if ((ll_neighbor < ob[i].size()) && (ob[i][neighbor_index].group != ob[i][current_index].group))
        {
          if (isTouched(ob[i][neighbor_index], ob[i][current_index]))
          {
            if (ob[i][neighbor_index].group == -1)
            {
              addShapeToGroup(ob[i][current_index].group, ob[i][neighbor_index]);
            }
            else
            {
              updateGroupIndex(ob[i][current_index], ob[i][neighbor_index]);
            }
          }
        }
      }

      current = ob[i][sort_ur_ob_[i][current].index].ul_index;
      int lr_neighbor = current + 1;
      if (lr_neighbor < ob[i].size())
      {
        current_index = sort_ul_ob_[i][current].index;
        neighbor_index = sort_ul_ob_[i][lr_neighbor].index;
        while ((lr_neighbor != ob[i].size()) &&
               ((ob[i][neighbor_index].ll_pt.x < ob[i][current_index].ll_pt.x) ||
                (upper_bound_.y-ob[i][neighbor_index].ll_pt.y) <= (upper_bound_.y-ob[i][current_index].ll_pt.y)))
        {
          lr_neighbor++;
          neighbor_index = sort_ul_ob_[i][lr_neighbor].index;
        }

        if ((lr_neighbor < ob[i].size()) && (ob[i][neighbor_index].group != ob[i][current_index].group))
        {
          if (isTouched(ob[i][neighbor_index], ob[i][current_index]))
          {
            if (ob[i][neighbor_index].group == -1)
            {
              addShapeToGroup(ob[i][current_index].group, ob[i][neighbor_index]);
            }
            else
            {
              updateGroupIndex(ob[i][current_index], ob[i][neighbor_index]);
            }
          }
        }
      }
    } // end shapes
  } // end each layer
}

void NetOpenFinder::updateGroupIndex(routed_shape& primary_shape, routed_shape& append_shape)
{
  rs_group* primary_group = &rs_groups_[primary_shape.layer-1][primary_shape.group];
  rs_group* append_group = &rs_groups_[append_shape.layer-1][append_shape.group];

  for (int i = 0; i < append_group->member.size(); i++)
  {
    append_group->member[i].group = primary_group->id;
    primary_group->member.push_back(append_group->member[i]);

    sort_layer_rs_[append_shape.layer-1][append_group->member[i].index].group = primary_group->id;
    sort_ll_rs_[append_shape.layer-1][append_group->member[i].ll_index].group = primary_group->id;
    sort_lr_rs_[append_shape.layer-1][append_group->member[i].lr_index].group = primary_group->id;
    sort_ul_rs_[append_shape.layer-1][append_group->member[i].ul_index].group = primary_group->id;
    sort_ur_rs_[append_shape.layer-1][append_group->member[i].ur_index].group = primary_group->id;
  }

  append_group->member.clear();
}

void NetOpenFinder::updateGroupIndex(obstacle& primary_shape, obstacle& append_shape)
{
  ob_group* primary_group = &ob_groups_[primary_shape.layer-1][primary_shape.group];
  ob_group* append_group = &ob_groups_[append_shape.layer-1][append_shape.group];

  for (int i = 0; i < append_group->member.size(); i++)
  {
    append_group->member[i].group = primary_group->id;
    primary_group->member.push_back(append_group->member[i]);

    sort_layer_ob_[append_shape.layer-1][append_group->member[i].index].group = primary_group->id;
    sort_ll_ob_[append_shape.layer-1][append_group->member[i].ll_index].group = primary_group->id;
    sort_lr_ob_[append_shape.layer-1][append_group->member[i].lr_index].group = primary_group->id;
    sort_ul_ob_[append_shape.layer-1][append_group->member[i].ul_index].group = primary_group->id;
    sort_ur_ob_[append_shape.layer-1][append_group->member[i].ur_index].group = primary_group->id;
  }

  append_group->member.clear();
}

void NetOpenFinder::addShapeToGroup(const int& group_index, routed_shape& rs)
{
  rs_groups_[rs.layer-1][group_index].member.push_back(rs);
  rs.group = group_index;
}

void NetOpenFinder::addShapeToGroup(const int& group_index, obstacle& ob)
{
  ob_groups_[ob.layer-1][group_index].member.push_back(ob);
  ob.group = group_index;
}

// i->layer j->group
void NetOpenFinder::connectClosestShape(int i, int j, int current_index, location loc)
{
  std::vector<int> obs;
  switch (loc)
  {
    // find UR
    case LL:
    {
      int neighbor = sort_layer_rs_[i][current_index].ll_index + 1;
      if (neighbor < sort_layer_rs_[i].size())
      {
        while (neighbor < sort_layer_rs_[i].size() &&
               (sort_ll_rs_[i][neighbor].ll_pt.x <= sort_layer_rs_[i][current_index].ll_pt.x ||
                sort_ll_rs_[i][neighbor].ll_pt.y < sort_layer_rs_[i][current_index].ll_pt.y))
        {
          neighbor++;
        }

        if (neighbor < sort_layer_rs_[i].size())
        {
          int  neighbor_index = sort_ll_rs_[i][neighbor].index;
          if (sort_layer_rs_[i][current_index].group != sort_layer_rs_[i][neighbor_index].group)
          {
            edge e = getClosestEdge(sort_layer_rs_[i][current_index], sort_layer_rs_[i][neighbor_index], LL);
//            if (!isCollision(e, obs))
            if (!isCollision(e))
            {
              int min, max;
              if (e.group_a <= e.group_b)
              {
                min = e.group_a;
                max = e.group_b;
              }
              else
              {
                min = e.group_b;
                max = e.group_a;
              }

              if (edge_table_[i][max][min].dist != (-1))
              {
                if (e.dist < edge_table_[i][max][min].dist)
                {
                  edge_table_[i][max][min] = e;
                }
              }
              else
              {
                edge_table_[i][max][min] = e;
              }



              rs_groups_[i][j].edges.push_back(e);
              rs_groups_[i][sort_layer_rs_[i][neighbor_index].group].edges.push_back(e);
            }
          }
        }
      }
      break;
    }
    // find UL
    case LR:
    {
      int neighbor = sort_layer_rs_[i][current_index].lr_index + 1;
      if (neighbor < sort_layer_rs_[i].size())
      {
        while (neighbor < sort_layer_rs_[i].size() &&
               ((upper_bound_.x-sort_lr_rs_[i][neighbor].ll_pt.x) < (upper_bound_.x-sort_layer_rs_[i][current_index].ll_pt.x) ||
                sort_lr_rs_[i][neighbor].ll_pt.y <= sort_layer_rs_[i][current_index].ll_pt.y))
        {
          neighbor++;
        }

        if (neighbor < sort_layer_rs_[i].size())
        {
          int neighbor_index = sort_lr_rs_[i][neighbor].index;
          if (sort_layer_rs_[i][current_index].group != sort_layer_rs_[i][neighbor_index].group)
          {
            edge e = getClosestEdge(sort_layer_rs_[i][current_index], sort_layer_rs_[i][neighbor_index], LR);
//            if (!isCollision(e, obs))
            if (!isCollision(e))
            {
              int min, max;
              if (e.group_a <= e.group_b)
              {
                min = e.group_a;
                max = e.group_b;
              }
              else
              {
                min = e.group_b;
                max = e.group_a;
              }

              if (edge_table_[i][max][min].dist != (-1))
              {
                if (e.dist < edge_table_[i][max][min].dist)
                {
                  edge_table_[i][max][min] = e;
                }
              }
              else
              {
                edge_table_[i][max][min] = e;
              }



              rs_groups_[i][j].edges.push_back(e);
              rs_groups_[i][sort_layer_rs_[i][neighbor_index].group].edges.push_back(e);
            }
          }
        }
      }
      break;
    }

    // find LR
    case UL:
    {
      int neighbor = sort_layer_rs_[i][current_index].ul_index + 1;
      if (neighbor < sort_layer_rs_[i].size())
      {
        while ((neighbor < sort_layer_rs_[i].size()) &&
               ((sort_ul_rs_[i][neighbor].ll_pt.x < sort_layer_rs_[i][current_index].ll_pt.x) ||
                (upper_bound_.y-sort_ul_rs_[i][neighbor].ll_pt.y) <= (upper_bound_.y-sort_layer_rs_[i][current_index].ll_pt.y)))
        {
          neighbor++;
        }
        if (neighbor < sort_layer_rs_[i].size())
        {
          int neighbor_index = sort_ul_rs_[i][neighbor].index;
          if (sort_layer_rs_[i][current_index].group != sort_layer_rs_[i][neighbor_index].group)
          {
            edge e = getClosestEdge(sort_layer_rs_[i][current_index], sort_layer_rs_[i][neighbor_index], UL);
//            if (!isCollision(e, obs))
            if (!isCollision(e))
            {
              int min, max;
              if (e.group_a <= e.group_b)
              {
                min = e.group_a;
                max = e.group_b;
              }
              else
              {
                min = e.group_b;
                max = e.group_a;
              }

              if (edge_table_[i][max][min].dist != (-1))
              {
                if (e.dist < edge_table_[i][max][min].dist)
                {
                  edge_table_[i][max][min] = e;
                }
              }
              else
              {
                edge_table_[i][max][min] = e;
              }



              rs_groups_[i][j].edges.push_back(e);
              rs_groups_[i][sort_layer_rs_[i][neighbor_index].group].edges.push_back(e);
            }
          }
        }
      }
      break;
    }
    case UR:
    {
      int neighbor = sort_layer_rs_[i][current_index].ur_index + 1;
      if (neighbor < sort_layer_rs_[i].size())
      {
        while ((neighbor < sort_layer_rs_[i].size()) &&
               ((upper_bound_.x-sort_ur_rs_[i][neighbor].ll_pt.x) <= (upper_bound_.x-sort_layer_rs_[i][current_index].ll_pt.x) ||
                (upper_bound_.y-sort_ur_rs_[i][neighbor].ll_pt.y) < (upper_bound_.y-sort_layer_rs_[i][current_index].ll_pt.y)))
        {
          neighbor++;
        }
        if (neighbor < sort_layer_rs_[i].size())
        {
          int neighbor_index = sort_ur_rs_[i][neighbor].index;
          if (sort_layer_rs_[i][current_index].group != sort_layer_rs_[i][neighbor_index].group)
          {
            edge e = getClosestEdge(sort_layer_rs_[i][current_index], sort_layer_rs_[i][neighbor_index], UR);
//            if (!isCollision(e, obs))
            if (!isCollision(e))
            {
              int min, max;
              if (e.group_a <= e.group_b)
              {
                min = e.group_a;
                max = e.group_b;
              }
              else
              {
                min = e.group_b;
                max = e.group_a;
              }

              if (edge_table_[i][max][min].dist != (-1))
              {
                if (e.dist < edge_table_[i][max][min].dist)
                {
                  edge_table_[i][max][min] = e;
                }
              }
              else
              {
                edge_table_[i][max][min] = e;
              }
              rs_groups_[i][j].edges.push_back(e);
              rs_groups_[i][sort_layer_rs_[i][neighbor_index].group].edges.push_back(e);
            }
          }
        }
      }
      break;
    }
  }
}

edge NetOpenFinder::getClosestEdge(const routed_shape& current_rs, const routed_shape& neighbor_rs, const location loc)
{
  edge e;
  int min_dist = std::numeric_limits<int>::max();
  point current_lr_pt, current_ul_pt, neighbor_lr_pt, neighbor_ul_pt;
  current_lr_pt.x = current_rs.ur_pt.x;
  current_lr_pt.y = current_rs.ll_pt.y;
  current_ul_pt.x = current_rs.ll_pt.x;
  current_ul_pt.y = current_rs.ur_pt.y;

  neighbor_lr_pt.x = neighbor_rs.ur_pt.x;
  neighbor_lr_pt.y = neighbor_rs.ll_pt.y;
  neighbor_ul_pt.x = neighbor_rs.ll_pt.x;
  neighbor_ul_pt.y = neighbor_rs.ur_pt.y;
  switch (loc)
  {
    // LL
    case LL:
    {
      int d = getDist(current_rs.ur_pt, neighbor_rs.ll_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ur_pt;
        e.shape_b_pt = neighbor_rs.ll_pt;
        min_dist = d;
      }

      d = getDist(current_rs.ur_pt, neighbor_ul_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ur_pt;
        e.shape_b_pt = neighbor_ul_pt;
        min_dist = d;
      }

      d = getDist(current_lr_pt, neighbor_rs.ll_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_lr_pt;
        e.shape_b_pt = neighbor_rs.ll_pt;
        min_dist = d;
      }

      d = getDist(current_lr_pt, neighbor_ul_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_lr_pt;
        e.shape_b_pt = neighbor_ul_pt;
        min_dist = d;
      }

      break;
    }
    // UL
    case UL:
    {
      int d = getDist(current_rs.ur_pt, neighbor_rs.ll_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ur_pt;
        e.shape_b_pt = neighbor_rs.ll_pt;
        min_dist = d;
      }

      d = getDist(current_rs.ur_pt, neighbor_ul_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ur_pt;
        e.shape_b_pt = neighbor_ul_pt;
        min_dist = d;
      }

      d = getDist(current_lr_pt, neighbor_rs.ll_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_lr_pt;
        e.shape_b_pt = neighbor_rs.ll_pt;
        min_dist = d;
      }

      d = getDist(current_lr_pt, neighbor_ul_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_lr_pt;
        e.shape_b_pt = neighbor_ul_pt;
        min_dist = d;
      }

      break;
    }
    // LR
    case LR:
    {
      int d = getDist(current_ul_pt, neighbor_rs.ur_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_ul_pt;
        e.shape_b_pt = neighbor_rs.ur_pt;
        min_dist = d;
      }

      d = getDist(current_ul_pt, neighbor_lr_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_ul_pt;
        e.shape_b_pt = neighbor_lr_pt;
        min_dist = d;
      }

      d = getDist(current_rs.ll_pt, neighbor_rs.ur_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ll_pt;
        e.shape_b_pt = neighbor_rs.ur_pt;
        min_dist = d;
      }

      d = getDist(current_rs.ll_pt, neighbor_lr_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ll_pt;
        e.shape_b_pt = neighbor_lr_pt;
        min_dist = d;
      }

      break;
    }
    // UR
    case UR:
    {
      int d = getDist(current_ul_pt, neighbor_rs.ur_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_ul_pt;
        e.shape_b_pt = neighbor_rs.ur_pt;
        min_dist = d;
      }

      d = getDist(current_ul_pt, neighbor_lr_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_ul_pt;
        e.shape_b_pt = neighbor_lr_pt;
        min_dist = d;
      }

      d = getDist(current_rs.ll_pt, neighbor_rs.ur_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ll_pt;
        e.shape_b_pt = neighbor_rs.ur_pt;
        min_dist = d;
      }

      d = getDist(current_rs.ll_pt, neighbor_lr_pt);
      if (d < min_dist)
      {
        e.shape_a_pt = current_rs.ll_pt;
        e.shape_b_pt = neighbor_lr_pt;
        min_dist = d;
      }

      break;
    }
  } // end switch

  e.layer = current_rs.layer;
  e.dist = min_dist;
  e.shape_a = current_rs.index;
  e.shape_b = neighbor_rs.index;
  e.group_a = current_rs.group;
  e.group_b = neighbor_rs.group;

  return e;
}

void NetOpenFinder::generateCollisionFreePath()
{
  for (int i = 0; i < 1; i++)
  {
    for (int j = 0; j < rs_groups_[i].size(); j++)
    {
      for (int k = 0; k < rs_groups_[i][j].edges.size(); k++)
      {
        int n_index = 0;
        for (int m = 0; m < rs_groups_[i][j].edges[k].pts.size(); m++)
        {
          int next = m+1;
          int h_v = -1;
          if (rs_groups_[i][j].edges[k].pts[m].x == rs_groups_[i][j].edges[k].pts[next].x)
          {
            if (rs_groups_[i][j].edges[k].pts[m].y > rs_groups_[i][j].edges[k].pts[next].y)
            {
              h_v = 0;
            }
            else
            {
              h_v = 1;
            }
          }
          else
          {
            if (rs_groups_[i][j].edges[k].pts[m].x > rs_groups_[i][j].edges[k].pts[next].x)
            {
              h_v = 2;
            }
            else
            {
              h_v = 3;
            }
          }

          for (int n = 0; n < sort_layer_ob_[i].size(); n++)
          {
//            std::cout << n << std::endl;
            if (h_v == 0 && n != n_index)
            {
              n_index = n;
              if (sort_layer_ob_[i][n].ll_pt.y < rs_groups_[i][j].edges[k].pts[m].y &&
                  sort_layer_ob_[i][n].ll_pt.y > rs_groups_[i][j].edges[k].pts[next].y &&
                  (sort_layer_ob_[i][n].ll_pt.x < rs_groups_[i][j].edges[k].pts[m].x &&
                   sort_layer_ob_[i][n].ur_pt.x > rs_groups_[i][j].edges[k].pts[m].x))
              {
                point intersect;
                intersect.x = rs_groups_[i][j].edges[k].pts[m].x;
                intersect.y = sort_layer_ob_[i][n].ur_pt.y;
                rs_groups_[i][j].edges[k].pts.insert(rs_groups_[i][j].edges[k].pts.begin()+m+1, intersect);

                if ((sort_layer_ob_[i][n].ur_pt.x-rs_groups_[i][j].edges[k].pts[m].x) >
                    (rs_groups_[i][j].edges[k].pts[m].x - sort_layer_ob_[i][n].ll_pt.x))
                {
                  // go left
                  point ob_corner;
                  ob_corner.x = sort_layer_ob_[i][n].ll_pt.x;
                  ob_corner.y = sort_layer_ob_[i][n].ur_pt.y;
                  rs_groups_[i][j].edges[k].pts.insert(rs_groups_[i][j].edges[k].pts.begin()+m+2, ob_corner);

                  ob_corner.x = sort_layer_ob_[i][n].ll_pt.x;
                  ob_corner.y = rs_groups_[i][j].edges[k].pts[m+3].y;
                  rs_groups_[i][j].edges[k].pts.insert(rs_groups_[i][j].edges[k].pts.begin()+m+3, ob_corner);
                }
                else
                {
                  // go right
                  point ob_corner;
                  ob_corner.x = sort_layer_ob_[i][n].ur_pt.x;
                  ob_corner.y = sort_layer_ob_[i][n].ur_pt.y;
                  rs_groups_[i][j].edges[k].pts.insert(rs_groups_[i][j].edges[k].pts.begin()+m+2, ob_corner);

                  ob_corner.x = sort_layer_ob_[i][n].ur_pt.x;
                  ob_corner.y = rs_groups_[i][j].edges[k].pts[m+3].y;
                  rs_groups_[i][j].edges[k].pts.insert(rs_groups_[i][j].edges[k].pts.begin()+m+3, ob_corner);
                }

                m = m+1;
                break;
              }
            }
            else
            {

            }
          }
        }



//        point corner;
//        corner.x = rs_groups_[i][j].edges[k].shape_b_pt.x;
//        corner.y = rs_groups_[i][j].edges[k].shape_a_pt.y;
//        rs_groups_[i][j].edges[k].pts.push_back(rs_groups_[i][j].edges[k].shape_a_pt);
//        rs_groups_[i][j].edges[k].pts.push_back(corner);
//        rs_groups_[i][j].edges[k].pts.push_back(rs_groups_[i][j].edges[k].shape_b_pt);
      }
    }
  }
}

bool NetOpenFinder::isCollision(edge e, std::vector<int>& obs)
{
  obs = searchObstacle(e);
  if (obs.size() > 0)
  {
    // find collision-free path
    return true;
  }
  else
  {
    return false;
  }
}

bool NetOpenFinder::isCollision(int rs_a_index, int rs_b_index, point edge_pt_a, point edge_pt_b, obstacle ob, point& ob_intersect_pt)
{
  if (edge_pt_a.x < edge_pt_b.x)
  {

  }

}

bool NetOpenFinder::isCollision(edge e)
{
  routed_shape rs_1, rs_2;
  point corner;
  corner.x = e.shape_b_pt.x;
  corner.y = e.shape_a_pt.y;

  if (corner.x == e.shape_a_pt.x)
  {
    if (e.shape_a_pt.y >= corner.y)
    {
      rs_1.ur_pt = e.shape_a_pt;
      rs_1.ll_pt = corner;
    }
    else
    {
      rs_1.ur_pt = corner;
      rs_1.ll_pt = e.shape_a_pt;
    }

    if (e.shape_b_pt.x >= corner.x)
    {
      rs_2.ur_pt = e.shape_b_pt;
      rs_2.ll_pt = corner;
    }
    else
    {
      rs_2.ur_pt = corner;
      rs_2.ll_pt = e.shape_b_pt;
    }
  }
  else
  {
    if (e.shape_b_pt.y >= corner.y)
    {
      rs_1.ur_pt = e.shape_b_pt;
      rs_1.ll_pt = corner;
    }
    else
    {
      rs_1.ur_pt = corner;
      rs_1.ll_pt = e.shape_b_pt;
    }

    if (e.shape_a_pt.x >= corner.x)
    {
      rs_2.ur_pt = e.shape_a_pt;
      rs_2.ll_pt = corner;
    }
    else
    {
      rs_2.ur_pt = corner;
      rs_2.ll_pt = e.shape_a_pt;
    }
  }

//  bool collide = false;
  for (int i = 0; i < sort_layer_ob_[e.layer-1].size(); i++)
  {
    routed_shape r;
    r.ur_pt = sort_layer_ob_[e.layer-1][i].ur_pt;
    r.ll_pt = sort_layer_ob_[e.layer-1][i].ll_pt;
    if (isTouched_2(rs_1, r) || isTouched_2(rs_2, r))
      return true;
  }
  return false;
}

bool NetOpenFinder::isCollision_2(edge e)
{
  routed_shape rs_1, rs_2;
  point corner;
  corner.x = e.shape_b_pt.x;
  corner.y = e.shape_a_pt.y;

  if (corner.x == e.shape_a_pt.x)
  {
    if (e.shape_a_pt.y >= corner.y)
    {
      rs_1.ur_pt = e.shape_a_pt;
      rs_1.ll_pt = corner;
    }
    else
    {
      rs_1.ur_pt = corner;
      rs_1.ll_pt = e.shape_a_pt;
    }

    if (e.shape_b_pt.x >= corner.x)
    {
      rs_2.ur_pt = e.shape_b_pt;
      rs_2.ll_pt = corner;
    }
    else
    {
      rs_2.ur_pt = corner;
      rs_2.ll_pt = e.shape_b_pt;
    }
  }
  else
  {
    if (e.shape_b_pt.y >= corner.y)
    {
      rs_1.ur_pt = e.shape_b_pt;
      rs_1.ll_pt = corner;
    }
    else
    {
      rs_1.ur_pt = corner;
      rs_1.ll_pt = e.shape_b_pt;
    }

    if (e.shape_a_pt.x >= corner.x)
    {
      rs_2.ur_pt = e.shape_a_pt;
      rs_2.ll_pt = corner;
    }
    else
    {
      rs_2.ur_pt = corner;
      rs_2.ll_pt = e.shape_a_pt;
    }
  }

//  bool collide = false;
  for (int i = 0; i < sort_layer_ob_[e.layer-1].size(); i++)
  {
    routed_shape r;
    r.ur_pt = sort_layer_ob_[e.layer-1][i].ur_pt;
    r.ll_pt = sort_layer_ob_[e.layer-1][i].ll_pt;
    if (isTouched_2(rs_1, r) || isTouched_2(rs_2, r))
      return true;
  }
  return false;
}


std::vector<int> NetOpenFinder::searchObstacle(const edge& e)
{
  std::vector<int> index;
  int min_x, max_x, min_y, max_y;
  int x_a_index, x_b_index, y_a_index, y_b_index;
  if (e.shape_a_pt.x <= e.shape_b_pt.x)
  {
    min_x = e.shape_a_pt.x;
    max_x = e.shape_b_pt.x;
    x_a_index = sort_layer_rs_[e.layer-1][e.shape_a].x_shape_index;
    x_b_index = sort_layer_rs_[e.layer-1][e.shape_b].x_shape_index;
  }
  else
  {
    min_x = e.shape_b_pt.x;
    max_x = e.shape_a_pt.x;
    x_b_index = sort_layer_rs_[e.layer-1][e.shape_a].x_shape_index;
    x_a_index = sort_layer_rs_[e.layer-1][e.shape_b].x_shape_index;
  }

  if (e.shape_a_pt.y <= e.shape_b_pt.y)
  {
    min_y = e.shape_a_pt.y;
    max_y = e.shape_b_pt.y;
    y_a_index = sort_layer_rs_[e.layer-1][e.shape_a].y_shape_index;
    y_b_index = sort_layer_rs_[e.layer-1][e.shape_b].y_shape_index;
  }
  else
  {
    min_y = e.shape_b_pt.y;
    max_y = e.shape_a_pt.y;
    y_b_index = sort_layer_rs_[e.layer-1][e.shape_a].y_shape_index;
    y_a_index = sort_layer_rs_[e.layer-1][e.shape_b].y_shape_index;
  }

  // search x dir
  int neighbor = x_a_index + 1;
  if (e.shape_a_pt.y < e.shape_b_pt.y)
  {
    while (sort_x_shapes_[e.layer-1][neighbor].ll_pt.x < sort_x_shapes_[e.layer-1][x_b_index].ll_pt.x)
    {
      if (sort_x_shapes_[e.layer-1][neighbor].type == 1 &&
          (sort_x_shapes_[e.layer-1][neighbor].ur_pt.y > e.shape_a_pt.y &&
           sort_x_shapes_[e.layer-1][neighbor].ll_pt.y < e.shape_a_pt.y))
      {

        index.push_back(neighbor);
      }
      neighbor++;
    }
  }
  else
  {
    while (sort_x_shapes_[e.layer-1][neighbor].ll_pt.x < sort_x_shapes_[e.layer-1][x_b_index].ll_pt.x)
    {
      if (sort_x_shapes_[e.layer-1][neighbor].type == 1 &&
          (sort_x_shapes_[e.layer-1][neighbor].ur_pt.y > e.shape_b_pt.y &&
           sort_x_shapes_[e.layer-1][neighbor].ll_pt.y < e.shape_b_pt.y))
      {
        index.push_back(neighbor);
      }
      neighbor++;
    }
  }

  // search y dir
  neighbor = y_a_index + 1;
  if (e.shape_a_pt.x < e.shape_b_pt.x)
  {
    while (sort_y_shapes_[e.layer-1][neighbor].ll_pt.y < sort_y_shapes_[e.layer-1][y_b_index].ll_pt.y)
    {
      if (sort_y_shapes_[e.layer-1][neighbor].type == 1 &&
          (sort_y_shapes_[e.layer-1][neighbor].ur_pt.x > e.shape_b_pt.x &&
           sort_y_shapes_[e.layer-1][neighbor].ll_pt.x < e.shape_b_pt.x))
      {
        index.push_back(neighbor);
      }
      neighbor++;
    }
  }
  else
  {
    while (sort_y_shapes_[e.layer-1][neighbor].ll_pt.y < sort_y_shapes_[e.layer-1][y_b_index].ll_pt.y)
    {
      if (sort_y_shapes_[e.layer-1][neighbor].type == 1 &&
          (sort_y_shapes_[e.layer-1][neighbor].ur_pt.x > e.shape_b_pt.x &&
           sort_y_shapes_[e.layer-1][neighbor].ll_pt.x < e.shape_b_pt.x))
      {
        index.push_back(neighbor);
      }
      neighbor++;
    }
  }

  return index;
}

//edge NetOpenFinder::findPath(routed_shape shapeA, routed_shape shapeB, point pointA, point pointB)
//{
//    edge e1;
//    edge e2;
//    point temp;
//    std::vector<int> obs1;
//    std::vector<int> obs2;

//    e1.point.push_back(pointA);
//    temp.x = pointB.x;
//    temp.y = pointA.y;
//    e1.point.push_back(temp);
//    e1.point.push_back(pointB);

//    e2.point.push_back(pointA);
//    temp.x = pointA.x;
//    temp.y = pointB.y;
//    e2.point.push_back(temp);
//    e2.point.push_back(pointB);

//    if(!isCollision(e1, obs1)) {
//        // compute distance
//        e1.shape_a = shapeA;
//        e1.shape_b = shapeB;
//        return e1;
//    } else if (!isCollision(e2, obs2)) {
//        // compute distance
//        e2.shape_a = shapeA;
//        e2.shape_b = shapeB;
//        return e2;
//    }

//    if(isCollision(e1.point[0], e1.point[1], obs1[0], temp))
//    {
//        e1.point.insert(e1.point.begin()+1, temp);
//        temp = closestPoint(temp, obs1[0]);
//        e1.point.insert(e1.point.begin()+2, temp);
//        // bug
//        point x;
//        x.x = temp.x;
//        x.y = e1.point[3].y;
//        e1.point.insert(e1.point.begin()+3, x);
//    }
//    else if (isCollision(e1.point[1], e1.point[2], obs1[0], temp))
//    {
//        e1.point.insert(e1.point.begin()+2, temp);
//        temp = closestPoint(temp, obs1[0]);
//        e1.point.insert(e1.point.begin()+3, temp);
//        // bug
//        point x;
//        x.x = temp.x;
//        x.y = e1.point[4].y;
//        e1.point.insert(e1.point.begin()+4, x);
//    } else {
//        return e1;
//    }

//    if(isCollision(e2.point[0], e2.point[1], obs2[0], temp))
//    {
//        e2.point.insert(e2.point.begin()+1, temp);
//        temp = closestPoint(temp, obs2[0]);
//        e2.point.insert(e2.point.begin()+2, temp);
//        // bug
//        point x;
//        x.x = temp.x;
//        x.y = e2.point[3].y;
//        e2.point.insert(e2.point.begin()+3, x);
//    } else if (isCollision(e2.point[1], e2.point[2], obs2[0], temp)) {
//        e2.point.insert(e2.point.begin()+2, temp);
//        temp = closestPoint(temp, obs1[0]);
//        e2.point.insert(e2.point.begin()+3, temp);
//        point x;
//        x.x = temp.x;
//        x.y = e2.point[4].y;
//        e2.point.insert(e2.point.begin()+4, x);
//    } else {
//        return e2;
//    }

//    e1.dist = computeDistance(e1);
//    e2.dist = computeDistance(e2);

//    if(e1.dist <= e2.dist) {
//        return e1;
//    } else {
//        return e2;
//    }

//}

bool NetOpenFinder::isTouched(const routed_shape& rs_a, const routed_shape& rs_b)
{
  if (rs_a.ur_pt.x >= rs_b.ll_pt.x && rs_a.ur_pt.y >= rs_b.ll_pt.y &&
      rs_b.ur_pt.x >= rs_a.ll_pt.x && rs_b.ur_pt.y >= rs_a.ll_pt.y)
    return true;
  else
    return false;
}

bool NetOpenFinder::isTouched_2(const routed_shape& rs_a, const routed_shape& rs_b)
{
  if (rs_a.ur_pt.x > rs_b.ll_pt.x && rs_a.ur_pt.y > rs_b.ll_pt.y &&
      rs_b.ur_pt.x > rs_a.ll_pt.x && rs_b.ur_pt.y > rs_a.ll_pt.y)
    return true;
  else
    return false;
}

bool NetOpenFinder::isTouched(const obstacle& ob_a, const obstacle& ob_b)
{
  if (ob_a.ur_pt.x >= ob_b.ll_pt.x && ob_a.ur_pt.y >= ob_b.ll_pt.y &&
      ob_b.ur_pt.x >= ob_a.ll_pt.x && ob_b.ur_pt.y >= ob_a.ll_pt.y)
    return true;
  else
    return false;
}

void NetOpenFinder::sortElementToLayer()
{
  for (auto i : raw_rs_)
  {
    sort_layer_rs_[i.layer-1].push_back(i);
  }

  for (auto i : raw_vias_)
  {
    sort_layer_rv_[i.layer-1].push_back(i);
  }

  for (auto i : raw_ob_)
  {
    sort_layer_ob_[i.layer-1].push_back(i);
  }
}

int NetOpenFinder::distToUR(const routed_shape& rs)
{
  return ((upper_bound_.x-rs.ll_pt.x)+(upper_bound_.y-rs.ll_pt.y));
}

int NetOpenFinder::distToUL(const routed_shape& rs)
{
  return ((rs.ll_pt.x-0)+(upper_bound_.y-rs.ll_pt.y));
}

int NetOpenFinder::distToLR(const routed_shape& rs)
{
  return ((upper_bound_.x-rs.ll_pt.x)+(rs.ll_pt.y-0));
}

int NetOpenFinder::distToUR(const obstacle& ob)
{
  return ((upper_bound_.x-ob.ll_pt.x)+(upper_bound_.y-ob.ll_pt.y));
}

int NetOpenFinder::distToUL(const obstacle& ob)
{
  return ((ob.ll_pt.x-0)+(upper_bound_.y-ob.ll_pt.y));
}

int NetOpenFinder::distToLR(const obstacle& ob)
{
  return ((upper_bound_.x-ob.ll_pt.x)+(ob.ll_pt.y-0));
}

point NetOpenFinder::closestPoint(point x, obstacle y)
{
    int min_dist = std::numeric_limits<int>::max();
    point min_point;

    point ll;
    point lr;
    point ur;
    point ul;

    ll.x = ob_groups_[y.layer-1][y.group].min_x;
    ll.y = ob_groups_[y.layer-1][y.group].min_y;

    lr.x = ob_groups_[y.layer-1][y.group].max_x;
    lr.y = ob_groups_[y.layer-1][y.group].min_y;

    ur.x = ob_groups_[y.layer-1][y.group].max_x;
    ur.y = ob_groups_[y.layer-1][y.group].max_y;

    ul.x = ob_groups_[y.layer-1][y.group].min_x;
    ul.y = ob_groups_[y.layer-1][y.group].max_y;

    if(getDist(x, ll) < min_dist) {
        min_dist = getDist(x, ll);
        min_point = ll;
    }

    if(getDist(x, lr) < min_dist) {
        min_dist = getDist(x, lr);
        min_point = lr;
    }

    if(getDist(x, ur) < min_dist) {
        min_dist = getDist(x, ur);
        min_point = ur;
    }

    if(getDist(x, ul) < min_dist) {
        min_dist = getDist(x, ul);
        min_point = ul;
    }

    return min_point;
}

int NetOpenFinder::getDist(const point& a, const point& b)
{
  int dist = 0;
  dist = std::abs(a.x-b.x)+std::abs(a.y-b.y);
  return dist;
}

int NetOpenFinder::computeEdgeDistance(edge e)
{
  int dist = 0;
  for(int i = 0; i < e.pts.size()-1; i++)
  {
      dist = dist + abs(e.pts[i].x - e.pts[i+1].x) +  abs(e.pts[i].y - e.pts[i+1].y);
  }
  return dist;
}
