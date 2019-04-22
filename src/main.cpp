#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

#include <net_open_finder.h>

std::unique_ptr<NetOpenFinder> read_file(const std::string& file_name);

void write_shape_to_file(const std::string& file_name, const std::vector<routed_shape> rs_);

void write_shape_to_file(const std::vector<int> index, const std::vector<routed_shape> rs);
void write_shape_to_file(const std::vector<int> index, const std::vector<obstacle> rs);

void write_edges_to_file(const std::vector<edge>& e);

void write_vias_to_file(const std::vector<routed_via>& v, const std::vector<edge>& e);

void write_line_to_file(const std::vector<line>& l);

// extract number from number inside bracket and return in vector (num1, num2)
point find_num_in_bracket(std::string word);

void print_usage()
{
  std::cout << "usage: ./net_open_finder <input_file>" << std::endl;
}

int main (int argc, char* argv[])
{
  if (argc != 2)
  {
    print_usage();
    return 1;
  }

  std::string input_file_name = argv[1];
  auto net_open_finder = read_file(input_file_name);

  auto routed_shapes = net_open_finder->getRoutedShapes();
//  auto obs = net_open_finder->getObstacle();
//  auto group_index = net_open_finder->getGroupIndex();


//  auto result = net_open_finder->solve();
  auto result = net_open_finder->solve_2();

  std::vector<edge> e;
  for (int i = 0; i < result.size(); i++)
  {
    for (int j = 0; j < result[i].size(); j++)
    {
      e.push_back(result[i][j]);
    }
  }

  net_open_finder->connectVias();

//  for (int i = 0; i < net_open_finder->edges_.size(); i++)
//    e.push_back(net_open_finder->edges_[i]);

//  write_shape_to_file(group_index, routed_shapes[0]);
//  write_shape_to_file(group_index, obs[0]);
//  write_edges_to_file(net_open_finder->edges_);
  write_edges_to_file(e);
  write_vias_to_file(net_open_finder->vias_, net_open_finder->via_edge_);
//  write_line_to_file(result);

  return 0;
}

std::unique_ptr<NetOpenFinder> read_file (const std::string& file_name)
{
  std::ifstream net_open_description(file_name.c_str());
  if (!net_open_description)
  {
    std::cout << "unable to open the file" << std::endl;
    exit(1);
  }

  std::string input_word;
  std::stringstream ss;
  std::string s1, s2;

  // get via cost
  int via_cost = 0;
  std::getline(net_open_description, input_word);
  ss << input_word;
  ss >> s1 >> s2 >> via_cost;
  ss = std::stringstream();

  // spacing
  int spacing = 0;
  std::getline(net_open_description, input_word);
  ss << input_word;
  ss >> s1 >> s2 >> spacing;
  ss = std::stringstream();

  // get boundary
  std::string ub, lb;
  point lower_bound, upper_bound;
  std::getline(net_open_description, input_word);
  ss << input_word;
  ss >> s1 >> s2 >> lb >> ub;
  lower_bound = find_num_in_bracket(lb);
  upper_bound = find_num_in_bracket(ub);
  ss = std::stringstream();

  // get number of metal layers
  int num_layer = 0;
  std::getline(net_open_description, input_word);
  ss << input_word;
  ss >> s1 >> s2 >> num_layer;
  ss = std::stringstream();

  // get number of routed shapes
  int num_routed_shape = 0;
  std::getline(net_open_description, input_word);
  ss << input_word;
  ss >> s1 >> s2 >> num_routed_shape;
  ss = std::stringstream();

  // get number of routed vias
  int num_routed_via = 0;
  std::getline(net_open_description, input_word);
  ss << input_word;
  ss >> s1 >> s2 >> num_routed_via;
  ss = std::stringstream();

  // get number of routed vias
  int num_obstacle = 0;
  std::getline(net_open_description, input_word);
  ss << input_word;
  ss >> s1 >> s2 >> num_obstacle;
  ss = std::stringstream();

  std::vector<routed_shape> routed_shapes;
  int loop_count = 0;
  while (loop_count < num_routed_shape)
  {
    std::getline(net_open_description, input_word);
    std::stringstream ss(input_word);

    std::string ll, ur;
    ss >> s1 >> s2 >> ll >> ur;

    routed_shape rs;
    rs.index = loop_count;
    rs.ll_pt = find_num_in_bracket(ll);
    rs.ur_pt = find_num_in_bracket(ur);
    rs.layer = std::atoi(s2.substr(1, s2.size()).c_str());
    routed_shapes.push_back(rs);

    loop_count++;
  }

  std::vector<routed_via> routed_vias;
  loop_count = 0;
  while (loop_count < num_routed_via)
  {
    std::getline(net_open_description, input_word);
    std::stringstream ss(input_word);

    std::string pos;
    ss >> s1 >> s2 >> pos;

    routed_via rv;
    rv.position = find_num_in_bracket(pos);
    rv.layer = std::atoi(s2.substr(1, s2.size()).c_str());
    routed_vias.push_back(rv);

    loop_count++;
  }

  std::vector<obstacle> obstacles;
  loop_count = 0;
  while (loop_count < num_obstacle)
  {
    std::getline(net_open_description, input_word);
    std::stringstream ss(input_word);

    std::string ll, ur;
    ss >> s1 >> s2 >> ll >> ur;

    obstacle ob;
    ob.index = loop_count;
    ob.ll_pt = find_num_in_bracket(ll);
    ob.ur_pt = find_num_in_bracket(ur);
    ob.layer = std::atoi(s2.substr(1, s2.size()).c_str());

    // extra spacing for obstacles
    ob.ll_pt.x -= spacing;
    ob.ll_pt.y -= spacing;
    ob.ur_pt.x += spacing;
    ob.ur_pt.y += spacing;

    obstacles.push_back(ob);

    loop_count++;
  }

  net_open_description.close();

  std::unique_ptr<NetOpenFinder> nof_(new NetOpenFinder(via_cost, spacing, num_layer, upper_bound, routed_shapes, routed_vias, obstacles));
  return nof_;
}

void write_line_to_file(const std::vector<line>& l)
{
  std::string file_name = "../output/lines.out";
  std::ofstream output_file(file_name.c_str());
  if (!output_file)
  {
    std::cout << "unable to write edge file" << std::endl;
  }

  for (int i = 0; i < l.size(); i++)
  {
    if (l[i].hv == 0)
    {
      output_file << "H-line M" << l[i].layer << " (" << l[i].pt_a.x << "," << l[i].pt_a.y
                  << ") (" << l[i].pt_b.x << "," << l[i].pt_b.y << ")\n";
    }
    else
    {
      output_file << "V-line M" << l[i].layer << " (" << l[i].pt_a.x << "," << l[i].pt_a.y
                << ") (" << l[i].pt_b.x << "," << l[i].pt_b.y << ")\n";
    }
  }
}

void write_shape_to_file(const std::string& file_name, const std::vector<routed_shape> rs_)
{
  std::ofstream output_file(file_name.c_str());
  if (!output_file)
  {
    std::cout << "unable to write file" << std::endl;
  }

  for (int i = 0; i < rs_.size(); i++)
  {
    output_file << "RoutedShape M";
    output_file << rs_[i].layer;
    output_file << " (" << rs_[i].ll_pt.x << "," << rs_[i].ll_pt.y << ")";
    output_file << " (" << rs_[i].ur_pt.x << "," << rs_[i].ur_pt.y << ")\n";
  }

  output_file.close();
}

void write_shape_to_file(const std::vector<int> index, const std::vector<routed_shape> rs)
{
  std::string file_name = "../output/test_line.out";
  std::ofstream output_file(file_name.c_str());
  if (!output_file)
  {
    std::cout << "unable to write file" << std::endl;
  }

  for (int i = 0; i < index.size(); i++)
  {
    output_file << "H-line M" << rs[index[i]].layer << " (" << rs[index[i]].ll_pt.x << "," << rs[index[i]].ll_pt.y
        << ") (" << rs[index[i]].ur_pt.x << "," << rs[index[i]].ll_pt.y << ")\n";
    output_file << "H-line M" << rs[index[i]].layer << " (" << rs[index[i]].ll_pt.x << "," << rs[index[i]].ur_pt.y
        << ") (" << rs[index[i]].ur_pt.x << "," << rs[index[i]].ur_pt.y << ")\n";
    output_file << "V-line M" << rs[index[i]].layer << " (" << rs[index[i]].ll_pt.x << "," << rs[index[i]].ll_pt.y
        << ") (" << rs[index[i]].ll_pt.x << "," << rs[index[i]].ur_pt.y << ")\n";
    output_file << "V-line M" << rs[index[i]].layer << " (" << rs[index[i]].ur_pt.x << "," << rs[index[i]].ll_pt.y
        << ") (" << rs[index[i]].ur_pt.x << "," << rs[index[i]].ur_pt.y << ")\n";
  }

  output_file.close();
}

void write_shape_to_file(const std::vector<int> index, const std::vector<obstacle> ob)
{
  std::string file_name = "../output/test_line.out";
  std::ofstream output_file(file_name.c_str());
  if (!output_file)
  {
    std::cout << "unable to write file" << std::endl;
  }

  for (int i = 0; i < index.size(); i++)
  {
    output_file << "H-line M" << ob[index[i]].layer << " (" << ob[index[i]].ll_pt.x << "," << ob[index[i]].ll_pt.y
        << ") (" << ob[index[i]].ur_pt.x << "," << ob[index[i]].ll_pt.y << ")\n";
    output_file << "H-line M" << ob[index[i]].layer << " (" << ob[index[i]].ll_pt.x << "," << ob[index[i]].ur_pt.y
        << ") (" << ob[index[i]].ur_pt.x << "," << ob[index[i]].ur_pt.y << ")\n";
    output_file << "V-line M" << ob[index[i]].layer << " (" << ob[index[i]].ll_pt.x << "," << ob[index[i]].ll_pt.y
        << ") (" << ob[index[i]].ll_pt.x << "," << ob[index[i]].ur_pt.y << ")\n";
    output_file << "V-line M" << ob[index[i]].layer << " (" << ob[index[i]].ur_pt.x << "," << ob[index[i]].ll_pt.y
        << ") (" << ob[index[i]].ur_pt.x << "," << ob[index[i]].ur_pt.y << ")\n";
  }

  output_file.close();
}

void write_edges_to_file(const std::vector<edge>& e)
{
  std::string file_name = "/home/shtseng/ICCAD2017/output/edges.out";
  std::ofstream output_file(file_name.c_str());
  if (!output_file)
  {
    std::cout << "unable to write edge file" << std::endl;
  }

  for (int i = 0; i < e.size(); i++)
  {
    if ((e[i].shape_a_pt.x- e[i].shape_b_pt.x) != 0)
    {
      output_file << "H-line M" << e[i].layer << " (" << e[i].shape_a_pt.x << "," << e[i].shape_a_pt.y
                  << ") (" << e[i].shape_b_pt.x << "," << e[i].shape_a_pt.y << ")\n";
    }

    if ((e[i].shape_a_pt.y- e[i].shape_b_pt.y) != 0)
    {
      output_file << "V-line M" << e[i].layer << " (" << e[i].shape_b_pt.x << "," << e[i].shape_a_pt.y
                << ") (" << e[i].shape_b_pt.x << "," << e[i].shape_b_pt.y << ")\n";
    }
   }

  output_file.close();
}

void write_vias_to_file(const std::vector<routed_via>& v, const std::vector<edge>& e)
{
  std::string file_name = "/home/shtseng/ICCAD2017/output/edges.out";
  std::ofstream output_file(file_name.c_str(), std::ofstream::app);
  if (!output_file)
  {
    std::cout << "unable to write edge file" << std::endl;
  }

  for (int i = 0; i < v.size(); i++)
  {
    output_file << "Vias V" << v[i].layer << " (" << v[i].position.x << "," << v[i].position.y << ")\n";
  }

  for (int i = 0; i < e.size(); i++)
  {
    if ((e[i].shape_a_pt.x- e[i].shape_b_pt.x) != 0)
    {
      output_file << "H-line M" << e[i].layer << " (" << e[i].shape_a_pt.x << "," << e[i].shape_a_pt.y
                  << ") (" << e[i].shape_b_pt.x << "," << e[i].shape_a_pt.y << ")\n";
    }

    if ((e[i].shape_a_pt.y- e[i].shape_b_pt.y) != 0)
    {
      output_file << "V-line M" << e[i].layer << " (" << e[i].shape_b_pt.x << "," << e[i].shape_a_pt.y
                << ") (" << e[i].shape_b_pt.x << "," << e[i].shape_b_pt.y << ")\n";
    }
  }

  output_file.close();
}

point find_num_in_bracket(std::string word)
{
  word = word.substr(1, word.size());
  word = word.substr(0, word.size()-1);

  std::size_t divide_pt = word.find(",");

  point coordinate;
  coordinate.x = std::atoi(word.substr(0, divide_pt).c_str());
  coordinate.y = std::atoi(word.substr(divide_pt+1, word.size()).c_str());

  return coordinate;
}
