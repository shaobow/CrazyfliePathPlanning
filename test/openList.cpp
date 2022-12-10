#include "openList.hpp"

namespace CF_PLAN {

openList::openList() {
  this->pq.clear();
  this->node_list.clear();
  this->umap.clear();
}

int openList::add_node(int x, int y, int z) {
  auto temp = make_unique<nodeDstar>(x, y, z);
  node_list.push_back(move(temp));
  return node_list.size() - 1;
}

bool openList::isSmallerkey(const pair<double, double>& key1,
                            const pair<double, double>& key2) {
  if (key1.first < key2.first) return true;
  if (key1.first == key2.first && key1.second < key2.second) return true;
  return false;
}

void openList::insert(const array<int, 3>& coord_u,
                      const pair<double, double>& key_u) {
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

void openList::remove(const array<int, 3>& coord_u) {
  auto itr = pq.find(coord_u);
  if (itr != pq.end()) pq.erase(itr);
}

std::array<int, 3> openList::top() {
  pair<double, double> tmp_key_top(DBL_MAX, DBL_MAX);
  array<int, 3> coord_top;

  for (auto itr = pq.begin(); itr != pq.end(); itr++) {
  }
}

pair<double, double> openList::topKey(const array<int, 3>& coord_top) {
  Idx idx_top = umap[coord_top];
  return node_list[idx_top].get()->get_key();
}

void openList::pop(const array<int, 3>& coord_top) { this->remove(coord_top); }

nodeDstar* openList::getNode(const array<int, 3>& coord_u) {
  Idx idx_u;
  if (umap.count(coord_u) == 0) {  // not in umap
    idx_u = add_node(coord_u[0], coord_u[1], coord_u[2]);
    umap[coord_u] = idx_u;
  } else {
    idx_u = umap[coord_u];
  }

  return node_list[idx_u].get();
}
}  // namespace CF_PLAN