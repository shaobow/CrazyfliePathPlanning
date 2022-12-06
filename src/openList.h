#ifndef OPENLIST_H__
#define OPENLIST_H__

#include <map>
#include <unordered_map>
#include <vector>

#include "nodeDstar.h"
#include "util.h"

using namespace std;
namespace CF_PLAN {

struct arrayHash {
  size_t operator()(const array<int, 3>& arr) const {
    size_t hx = std::hash<int>{}(arr[0]);
    size_t hy = std::hash<int>{}(arr[1]) << 1;
    size_t hz = std::hash<int>{}(arr[2]) << 2;
    return (hx ^ hy) ^ hz;
  }
};

class openList {
 private:
  unordered_map<array<int, 3>, pair<double, double>, arrayHash> pq;

  bool isSmallerKey(const pair<double, double>& lhs,
                    const pair<double, double>& rhs) {
    if (lhs.first < rhs.first) return true;
    if (lhs.first == rhs.first && lhs.second < rhs.second) return true;
    return false;
  }

 public:
  vector<unique_ptr<nodeDstar>> node_list;
  unordered_map<array<int, 3>, int, arrayHash> umap;

  int add_node(int x, int y, int z) {
    auto temp = make_unique<nodeDstar>(x, y, z);
    node_list.push_back(move(temp));
    return node_list.size() - 1;
  }

  openList(){};

  ~openList() = default;

  // void insert(array<int, 3> u, pair<double, double> key_u) {
  //   auto itr = pq.find(u);
  //   Idx idx_new;

  //   cout << "result: " << (itr != pq.end()) << endl;

  //   if (itr != pq.end()) {
  //     cout << "situation 3" << endl;
  //     array<int, 3> coord_exit = itr->first;
  //     umap[coord_exit] = idx_new;
  //     node_list[idx_new].get()->set_key(key_u);

  //     itr->second = key_u;
  //   } else {
  //     cout << "situation 1 or 2" << endl;

  //     if (umap.count(u) == 0) {
  //       cout << "situation 2" << endl;
  //       idx_new = add_node(u[0], u[1], u[2]);
  //     }

  //     umap[u] = idx_new;
  //     node_list[idx_new].get()->set_key(key_u);

  //     cout << "idx_new key: " << node_list[idx_new].get()->get_key().first
  //          << ", " << node_list[idx_new].get()->get_key().second << endl;

  //     pq.insert(make_pair(u, key_u));
  //   }
  // }

  void insert(array<int, 3> u, pair<double, double> key_u) {
    auto itr = pq.find(u);
    Idx idx_new;

    if (itr != pq.end()) {
      // cout << "situation 3" << endl;

      umap[u] = idx_new;
      node_list[idx_new].get()->set_key(key_u);

      itr->second = key_u;  // update key value
    } else {
      if (umap.count(u) == 0) {
        // cout << "situation 2" << endl;

        idx_new = add_node(u[0], u[1], u[2]);  // in node_list
        umap[u] = idx_new;
      } else {
        // cout << "situation 1" << endl;

        idx_new = umap[u];
      }

      node_list[idx_new].get()->set_key(key_u);
      pq.insert(make_pair(u, key_u));
    }
  }

  void remove(array<int, 3> u) {
    auto itr = pq.find(u);
    if (itr != pq.end()) {  // u within U
      pq.erase(itr);
    }
  }

  bool isInOpenList(array<int, 3> u) {
    auto itr = pq.find(u);
    return itr != pq.end();
  }

  array<int, 3> top() {
    // find min key
    pair<double, double> tmp_key = make_pair(DBL_MAX, DBL_MAX);
    array<int, 3> coord_top;

    for (auto itr = pq.begin(); itr != pq.end(); itr++) {
      if (isSmallerKey(itr->second, tmp_key)) {
        coord_top = itr->first;
        tmp_key = itr->second;
      }
    }

    return coord_top;
  }

  pair<double, double> topKey() {
    array<int, 3> coord_top = this->top();
    Idx idx_top = umap[coord_top];
    return node_list[idx_top].get()->get_key();
  }

  // alternative of topKey() to avoid search twice
  pair<double, double> topKey(array<int, 3> coord_top) {
    Idx idx_top = umap[coord_top];
    return node_list[idx_top].get()->get_key();
  }

  void pop() {
    array<int, 3> coord_top = this->top();
    this->remove(coord_top);
  }

  void pop(array<int, 3> coord_top) { this->remove(coord_top); }

  void clear() {
    pq.clear();
    node_list.clear();
  }

  bool empty() { return pq.size() == 0; }

  nodeDstar* getNode(array<int, 3> coord_u) {
    Idx idx_u;
    if (umap.count(coord_u) == 0) {
      idx_u = add_node(coord_u[0], coord_u[1], coord_u[2]);
      umap[coord_u] = idx_u;
    } else {
      idx_u = umap[coord_u];
    }

    return node_list[idx_u].get();
  }
};  // namespace CF_PLAN
}  // namespace CF_PLAN
#endif