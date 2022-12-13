#ifndef OPENLIST_H__
#define OPENLIST_H__

#include <map>
#include <unordered_map>
#include <vector>

#include "nodeDstar.h"
#include "util.h"

using namespace std;
namespace CF_PLAN {

class openList {
 private:
  unordered_map<array<int, 3>, pair<double, double>, arrayHash> pq;
  // map<array<int, 3>, pair<double, double>, arrayCompare> pq;

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

  openList() {
    // cout << "OL.clear()" << endl;

    pq.clear();
    node_list.clear();
    umap.clear();
  };

  ~openList() = default;

  void insert(array<int, 3> u, pair<double, double> key_u) {
    // cout << "OL.insert()-> find()" << endl;

    auto itr = pq.find(u);
    Idx idx_new;

    // cout << "OL.insert()-> end()" << endl;

    if (itr != pq.end()) {
      idx_new = umap[u];
      node_list[idx_new].get()->set_key(key_u);

      itr->second = key_u;  // update key value
    } else {
      if (umap.count(u) == 0) {
        idx_new = add_node(u[0], u[1], u[2]);
        umap[u] = idx_new;
      } else {
        idx_new = umap[u];
      }

      node_list[idx_new].get()->set_key(key_u);
      pq.insert(make_pair(u, key_u));
    }
  }

  void isInOpenList_and_remove(array<int, 3> u) {
    // cout << "OL.isInOpenList_and_remove()-> find()" << endl;

    auto itr = pq.find(u);

    // cout << "OL.isInOpenList_and_remove()-> end()" << endl;
    if (itr != pq.end()) {  // in openlist and removed
      pq.erase(itr);
    }
  }

  array<int, 3> top() {
    // find min key
    pair<double, double> tmp_key = make_pair(DBL_MAX, DBL_MAX);
    // array<int, 3> coord_top;
    array<int, 3> coord_top = pq.begin()->first;

    // cout << "OL.top()-> outside of loop" << endl;

    for (auto itr = pq.begin(); itr != pq.end(); itr++) {
      // cout << "OL.top()-> inside of loop" << endl;

      if (isSmallerKey(itr->second, tmp_key)) {
        coord_top = itr->first;
        tmp_key = itr->second;
      }
    }

    // cout << "top: ";
    // print_coord(coord_top);

    return coord_top;
  }

  // alternative of topKey() to avoid search twice
  pair<double, double> topKey(array<int, 3> coord_top) {
    Idx idx_top = umap[coord_top];
    return node_list[idx_top].get()->get_key();
  }

  void pop(array<int, 3> coord_top) {
    // cout << "pop: ";
    // print_coord(coord_top);
    // cout << "OL.pop() -> find()" << endl;

    // if (pq.find(coord_top) == pq.end())
    //   cout << "** something wrong when erase() **" << endl;

    // cout << "OL.pop() -> erase()" << endl;

    // pq.erase(itr);
    pq.erase(coord_top);

    // cout << "OL.pop() -> over" << endl;

    // pq.erase(pq.find(coord_top));
  }

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

  /* TEST  FUNCTION */

  void print_coord(array<int, 3> coord_u) {
    cout << "node (" << coord_u[0] << ", " << coord_u[1] << ", " << coord_u[2]
         << ") " << endl;
  }

  // void printNodeKey(array<int, 3> coord_u) {
  //   if (umap.count(coord_u) == 0)
  //     cout << "node (" << coord_u[0] << ", " << coord_u[1] << ", " <<
  //     coord_u[2]
  //          << ") doesn't exit" << endl;
  //   else {
  //     cout << "node (" << coord_u[0] << ", " << coord_u[1] << ", " <<
  //     coord_u[2]
  //          << ") ";
  //     node_list[umap[coord_u]].get()->print_key();
  //   }
  // }

  // void printAroundNode(array<int, 3> coord_u) {
  //   printNodeKey(coord_u);
  //   for (int dir = 0; dir < NUMOFDIRS; dir++) {
  //     int predX = coord_u[0] + dX[dir];
  //     int predY = coord_u[1] + dY[dir];
  //     int predZ = coord_u[2] + dZ[dir];

  //     printNodeKey({predX, predY, predZ});
  //   }
  //   cout << endl;
  // }

  // void isGEqualRhs(array<int, 3> coord_u) {
  //   nodeDstar* node_u = node_list[umap[coord_u]].get();

  //   if (node_u->get_g_value() != node_u->get_rhs_value()) {
  //     cout << "node (" << coord_u[0] << ", " << coord_u[1] << ", " <<
  //     coord_u[2]
  //          << ") inconsistent while removed. g: " << node_u->get_g_value()
  //          << ", rhs: " << node_u->get_rhs_value() << ", U.remove()" << endl;
  //     printAroundNode(coord_u);
  //   }
  // }
};
}  // namespace CF_PLAN
#endif