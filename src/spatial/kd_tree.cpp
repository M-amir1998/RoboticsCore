#include "roboticscore/spatial/kd_tree.hpp"

#include <algorithm>
#include <limits>
#include <numeric>
#include <utility>

#include "roboticscore/core/error.hpp"

namespace roboticscore::spatial {
namespace {

// Strict weak order on (distance, id) used for deterministic tie-breaking.
bool closer(const Neighbor& a, const Neighbor& b) {
  return a.distance_squared < b.distance_squared
         || (a.distance_squared == b.distance_squared && a.id < b.id);
}

// Iterative traversal state: node to visit and a lower bound on the squared distance from
// the query to any point in that subtree.
struct Pending {
  int node;
  double bound;
};

}  // namespace

// Invariant: for a node with split axis a, every point in the left subtree has coordinate
// <= node[a] and every point in the right subtree has coordinate >= node[a]. This is all the
// pruning bound (q[a] - node[a])^2 requires, and it tolerates duplicate coordinates.

KdTree2D::KdTree2D(std::span<const Vec2> points) : points_(points.begin(), points.end()) {
  rebuild();
}

std::size_t KdTree2D::insert(const Vec2& p) {
  const auto id = static_cast<int>(points_.size());
  points_.push_back(p);
  nodes_.push_back(Node{});
  if (root_ == kNone) {
    root_ = id;
    return points_.size() - 1;
  }
  int current = root_;
  int depth = 0;
  while (true) {
    Node& n = nodes_[static_cast<std::size_t>(current)];
    const int axis = n.axis;
    int& child = p(axis) < points_[static_cast<std::size_t>(current)](axis) ? n.left : n.right;
    ++depth;
    if (child == kNone) {
      child = id;
      nodes_[static_cast<std::size_t>(id)].axis = depth % 2;
      break;
    }
    current = child;
  }
  return points_.size() - 1;
}

int KdTree2D::build(std::vector<int>& ids, std::size_t begin, std::size_t end, int depth) {
  if (begin >= end) {
    return kNone;
  }
  const int axis = depth % 2;
  const std::size_t mid = begin + (end - begin) / 2;
  const auto first = ids.begin() + static_cast<std::ptrdiff_t>(begin);
  std::nth_element(first, ids.begin() + static_cast<std::ptrdiff_t>(mid),
                   ids.begin() + static_cast<std::ptrdiff_t>(end), [&](int a, int b) {
                     const double ca = points_[static_cast<std::size_t>(a)](axis);
                     const double cb = points_[static_cast<std::size_t>(b)](axis);
                     return ca < cb || (ca == cb && a < b);
                   });
  const int node = ids[mid];
  Node& n = nodes_[static_cast<std::size_t>(node)];
  n.axis = axis;
  n.left = build(ids, begin, mid, depth + 1);
  nodes_[static_cast<std::size_t>(node)].right = build(ids, mid + 1, end, depth + 1);
  return node;
}

void KdTree2D::rebuild() {
  nodes_.assign(points_.size(), Node{});
  std::vector<int> ids(points_.size());
  std::iota(ids.begin(), ids.end(), 0);
  root_ = build(ids, 0, ids.size(), 0);
}

void KdTree2D::clear() {
  points_.clear();
  nodes_.clear();
  root_ = kNone;
}

std::optional<Neighbor> KdTree2D::nearest(const Vec2& q) const {
  if (root_ == kNone) {
    return std::nullopt;
  }
  Neighbor best{0, std::numeric_limits<double>::infinity()};
  std::vector<Pending> stack;
  stack.push_back({root_, 0.0});
  while (!stack.empty()) {
    const Pending item = stack.back();
    stack.pop_back();
    if (item.bound > best.distance_squared) {
      continue;
    }
    const auto idx = static_cast<std::size_t>(item.node);
    const Neighbor candidate{idx, (points_[idx] - q).squaredNorm()};
    if (closer(candidate, best)) {
      best = candidate;
    }
    const Node& n = nodes_[idx];
    const double diff = q(n.axis) - points_[idx](n.axis);
    const int near_child = diff < 0.0 ? n.left : n.right;
    const int far_child = diff < 0.0 ? n.right : n.left;
    if (far_child != kNone) {
      stack.push_back({far_child, std::max(item.bound, diff * diff)});
    }
    if (near_child != kNone) {
      stack.push_back({near_child, item.bound});
    }
  }
  return best;
}

std::vector<Neighbor> KdTree2D::k_nearest(const Vec2& q, std::size_t k) const {
  std::vector<Neighbor> heap;  // max-heap under `closer`: front is the worst kept neighbour
  if (k == 0 || root_ == kNone) {
    return heap;
  }
  heap.reserve(k + 1);
  std::vector<Pending> stack;
  stack.push_back({root_, 0.0});
  while (!stack.empty()) {
    const Pending item = stack.back();
    stack.pop_back();
    if (heap.size() == k && item.bound > heap.front().distance_squared) {
      continue;
    }
    const auto idx = static_cast<std::size_t>(item.node);
    const Neighbor candidate{idx, (points_[idx] - q).squaredNorm()};
    if (heap.size() < k) {
      heap.push_back(candidate);
      std::push_heap(heap.begin(), heap.end(), closer);
    } else if (closer(candidate, heap.front())) {
      std::pop_heap(heap.begin(), heap.end(), closer);
      heap.back() = candidate;
      std::push_heap(heap.begin(), heap.end(), closer);
    }
    const Node& n = nodes_[idx];
    const double diff = q(n.axis) - points_[idx](n.axis);
    const int near_child = diff < 0.0 ? n.left : n.right;
    const int far_child = diff < 0.0 ? n.right : n.left;
    if (far_child != kNone) {
      stack.push_back({far_child, std::max(item.bound, diff * diff)});
    }
    if (near_child != kNone) {
      stack.push_back({near_child, item.bound});
    }
  }
  std::sort_heap(heap.begin(), heap.end(), closer);
  return heap;
}

std::vector<Neighbor> KdTree2D::radius_search(const Vec2& q, double radius) const {
  detail::require(radius >= 0.0, "KdTree2D::radius_search: radius must be non-negative");
  std::vector<Neighbor> result;
  if (root_ == kNone) {
    return result;
  }
  const double r2 = radius * radius;
  std::vector<Pending> stack;
  stack.push_back({root_, 0.0});
  while (!stack.empty()) {
    const Pending item = stack.back();
    stack.pop_back();
    if (item.bound > r2) {
      continue;
    }
    const auto idx = static_cast<std::size_t>(item.node);
    const double d2 = (points_[idx] - q).squaredNorm();
    if (d2 <= r2) {
      result.push_back({idx, d2});
    }
    const Node& n = nodes_[idx];
    const double diff = q(n.axis) - points_[idx](n.axis);
    const int near_child = diff < 0.0 ? n.left : n.right;
    const int far_child = diff < 0.0 ? n.right : n.left;
    if (far_child != kNone) {
      stack.push_back({far_child, std::max(item.bound, diff * diff)});
    }
    if (near_child != kNone) {
      stack.push_back({near_child, item.bound});
    }
  }
  std::sort(result.begin(), result.end(), closer);
  return result;
}

std::size_t KdTree2D::depth() const {
  if (root_ == kNone) {
    return 0;
  }
  std::size_t max_depth = 0;
  std::vector<std::pair<int, std::size_t>> stack{{root_, 1}};
  while (!stack.empty()) {
    const auto [node, d] = stack.back();
    stack.pop_back();
    max_depth = std::max(max_depth, d);
    const Node& n = nodes_[static_cast<std::size_t>(node)];
    if (n.left != kNone) {
      stack.emplace_back(n.left, d + 1);
    }
    if (n.right != kNone) {
      stack.emplace_back(n.right, d + 1);
    }
  }
  return max_depth;
}

}  // namespace roboticscore::spatial
