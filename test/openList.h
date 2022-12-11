#ifndef OPENLIST_H__
#define OPENLIST_H__

#include <memory>
#include <unordered_map>
#include <vector>

#include "nodeDstar.h"
#include "util.h"

using namespace std;
namespace CF_PLAN {

class openList {
 private:
  unordered_map<array<int, 3>, pair<double, double>, arrayHash> pq;

  int add_node(int x, int y, int z) {
    auto temp = make_unique<nodeDstar>(x, y, z);
    node_list.push_back(move(temp));
    return node_list.size() - 1;
  }

 public:
  vector<std::unique_ptr<nodeDstar>> node_list;
  unordered_map<array<int, 3>, int, arrayHash> umap;

  openList() {
    this->pq.clear();
    this->node_list.clear();
    this->umap.clear();
  };

  ~openList() = default;

  void insert(const array<int, 3>& coord_u, const pair<double, double>& key_u) {
    auto itr = pq.find(coord_u);
    Idx idx_u;

    if (itr != pq.end()) {  // (1) in pq, in umap
      umap[coord_u] = idx_u;
      node_list[idx_u].get()->set_key(
          key_u);  // update in node_listreturn itr != pq.end();

      itr->second = key_u;  // also update in pq
    } else {
      if (umap.count(coord_u) == 0) {  // (2) not in pq, not in umap
        idx_u = add_node(coord_u[0], coord_u[1], coord_u[2]);
        umap[coord_u] = idx_u;
      } else {  // (3) not in pq, in umap
        idx_u = umap[coord_u];
      }

      node_list[idx_u].get()->set_key(key_u);
      pq[coord_u] = key_u;  // insert in pq
    }
  }

  void remove(const array<int, 3>& coord_u) {
    auto itr = pq.find(coord_u);
    if (itr != pq.end()) pq.erase(itr);
  }

  std::array<int, 3> top() {
    // find min key
    pair<double, double> tmp_key_top(DBL_MAX, DBL_MAX);
    array<int, 3> coord_top;

    for (auto itr = pq.begin(); itr != pq.end(); itr++) {
      if (isSmallerkey(itr->second, tmp_key_top)) {
        coord_top = itr->first;
        tmp_key_top = itr->second;
      }
    }

    return coord_top;
  }

  pair<double, double> topKey(const array<int, 3>& coord_top) {
    Idx idx_top = umap[coord_top];
    return node_list[idx_top].get()->get_key();
  }

  void pop(const array<int, 3>& coord_top) { this->remove(coord_top); }

  nodeDstar* getNode(const array<int, 3>& coord_u) {
    Idx idx_u;
    if (umap.count(coord_u) == 0) {  // not in umap
      idx_u = add_node(coord_u[0], coord_u[1], coord_u[2]);
      umap[coord_u] = idx_u;
    } else {
      idx_u = umap[coord_u];
    }

    return node_list[idx_u].get();
  }

  bool isSmallerkey(const pair<double, double>& key1,
                    const pair<double, double>& key2) {
    if (key1.first < key2.first) return true;
    if (key1.first == key2.first && key1.second < key2.second) return true;
    return false;
  }
};
}  // namespace CF_PLAN

#endif