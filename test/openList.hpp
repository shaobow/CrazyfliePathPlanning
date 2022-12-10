#ifndef OPENLIST_H__
#define OPENLIST_H__

#include <memory>
#include <unordered_map>
#include <vector>

#include "nodeDstar.hpp"
#include "util.h"

using namespace std;
namespace CF_PLAN {
class openList {
 private:
  unordered_map<array<int, 3>, pair<double, double>> pq;

  int add_node(int x, int y, int z);

 public:
  vector<std::unique_ptr<nodeDstar>> node_list;
  unordered_map<array<int, 3>, int, arrayHash> umap;

  openList();
  ~openList() = default;

  void insert(const array<int, 3>& coord_u, const pair<double, double>& key_u);
  void remove(const array<int, 3>& coord_u);
  std::array<int, 3> top();
  pair<double, double> topKey(const array<int, 3>& coord_top);
  void pop(const array<int, 3>& coord_top);

  nodeDstar* getNode(const array<int, 3>& coord_u);
  bool isSmallerkey(const pair<double, double>& key1,
                    const pair<double, double>& key2);
};
}  // namespace CF_PLAN

#endif