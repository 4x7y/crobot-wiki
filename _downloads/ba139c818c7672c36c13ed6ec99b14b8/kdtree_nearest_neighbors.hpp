#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include "crobot/math/matrix.hpp"
#include "crobot/math/metric.hpp"
#include "crobot/math/nearest_neighbors.hpp"
#include "crobot/math/state_space.hpp"

namespace crobot::math {

template <typename T>
class KdtreeNearestNeighbors : public NearestNeighbors<T> {
private:
  struct Node;

public:
  using Neighbor       = std::pair<double, T>;
  using ConstMetricPtr = std::shared_ptr<const Metric<T>>;

  explicit KdtreeNearestNeighbors(ConstMetricPtr metric)
    : m_checks(),
      m_mean(),
      m_metric(metric),
      m_root(),
      m_samples(100),
      m_values(0),
      m_var() {
  }

  template <typename InputIterator>
  KdtreeNearestNeighbors(
      InputIterator first, InputIterator last, ConstMetricPtr metric)
    : m_checks(),
      m_mean(),
      m_metric(metric),
      m_root(),
      m_samples(100),
      m_values(0),
      m_var() {
    insert(first, last);
  }

  virtual ~KdtreeNearestNeighbors() = default;

  // Documentation inherited
  void clear() override {
    m_root.children[0].reset(nullptr);
    m_root.children[1].reset(nullptr);
    m_root.data.reset();
    m_values = 0;
  }

  // Documentation inherited
  bool empty() const {
    return !m_root.data && nullptr == m_root.children[0]
           && nullptr == m_root.children[1];
  }

  template <typename InputIterator>
  void insert(InputIterator first, InputIterator last) {
    if (empty()) {
      std::size_t size = std::distance(first, last);

      if (size > 1) {
        m_divide(m_root, first, last);
      }
      else {
        m_root.data = *first;
      }

      m_values += std::distance(first, last);
    }
    else {
      for (InputIterator i = first; i != last; ++i) {
        push(*i);
      }
    }
  }

  // Documentation inherited
  std::vector<Neighbor> nearest(
      const T& query, std::size_t k, bool sorted = true) const override {
    return search(query, &k, nullptr, sorted);
  }

  // Documentation inherited
  void push(const T& value) override {
    push(m_root, value);
    ++m_values;
  }

  // Documentation inherited
  std::vector<Neighbor> radius(
      const T& query, double radius, bool sorted = true) const override {
    return search(query, nullptr, &radius, sorted);
  }

  // Documentation inherited
  std::size_t size() const override {
    return m_values;
  }

  std::vector<T> data() const {
    std::vector<T> data;
    data.reserve(m_values);
    m_data(m_root, data);
    return data;
  }

  void set_checks(const std::optional<std::size_t>& checks) {
    m_checks = checks;
  }

  void set_samples(const std::size_t& samples) {
    m_samples = samples;
  }

  std::optional<std::size_t> get_checks() const {
    return m_checks;
  }

  std::size_t get_samples() const {
    return m_samples;
  }

  void swap(KdtreeNearestNeighbors& other) {
    using std::swap;
    swap(m_mean, other.mean);
    swap(m_metric, other.metric);
    swap(m_samples, other.samples);
    swap(m_root, other.root);
    swap(m_values, other.values);
    swap(m_var, other.var);
  }

  friend void swap(KdtreeNearestNeighbors& lhs, KdtreeNearestNeighbors& rhs) {
    lhs.swap(rhs);
  }

protected:
private:
  struct Branch
  {
    Branch(double& dist, std::vector<double>& sidedist, const Node* node)
      : dist(dist), node(node), sidedist(sidedist) {
    }

    Branch(double& dist, std::vector<double>&& sidedist, const Node* node)
      : dist(dist), node(node), sidedist(std::move(sidedist)) {
    }

    double dist;

    const Node* node;

    std::vector<double> sidedist;
  };

  struct BranchCompare
  {
    bool operator()(const Branch& lhs, const Branch& rhs) const {
      return lhs.dist > rhs.dist;
    }
  };

  struct Cut
  {
    bool operator()(const T& other) const {
      using std::begin;
      return *(begin(other) + index) < value;
    }

    std::size_t index;

    double value;
  };

  struct NeighborCompare
  {
    bool operator()(const Neighbor& lhs, const Neighbor& rhs) const {
      return lhs.first < rhs.first;
    }
  };

  struct Node
  {
    Node() : children(), cut(), data() {
    }

    ~Node() {
    }

    void swap(Node& other) {
      using std::swap;
      swap(children, other.children);
      swap(cut, other.cut);
      swap(data, other.data);
    }

    friend void swap(Node& lhs, Node& rhs) {
      lhs.swap(rhs);
    }

    std::array<std::unique_ptr<Node>, 2> children;

    Cut cut;

    std::optional<T> data;
  };

  void data(const Node& node, std::vector<T>& data) const {
    data.push_back(*node.data);

    for (std::size_t i = 0; i < node.children.size(); ++i) {
      m_data(*node.children[i], data);
    }
  }

  template <typename InputIterator>
  void divide(Node& node, InputIterator first, InputIterator last) {
    node.cut            = m_select(first, last);
    InputIterator split = std::partition(first, last, node.cut);

    for (std::size_t i = 0; i < 2; ++i) {
      node.children[i] = std::make_unique<Node>();

      InputIterator begin = 0 == i ? first : split;
      InputIterator end   = 0 == i ? split : last;

      if (std::distance(begin, end) > 1) {
        m_divide(*node.children[i], begin, end);
      }
      else {
        node.children[i]->data = *begin;
      }
    }
  }

  void push(Node& node, const T& value) {
    using std::begin;

    if (nullptr == node.children[0] && //
        nullptr == node.children[1] && //
        !node.data) {
      node.data = value;
    }
    else if (nullptr == node.children[0] && nullptr == node.children[1]) {
      std::size_t dim = value.size();
      double max      = 0.0;

      for (std::size_t i = 0; i < dim; ++i) {
        double span = std::abs(*(begin(value) + i) - *(begin(*node.data) + i));

        if (span > max) {
          max            = span;
          node.cut.index = i;
        }
      }

      for (std::size_t i = 0; i < 2; ++i) {
        node.children[i] = std::make_unique<Node>();
      }

      bool less = *(begin(value) + node.cut.index)
                  < *(begin(*node.data) + node.cut.index);
      node.cut.value = (*(begin(value) + node.cut.index)
                        + *(begin(*node.data) + node.cut.index))
                       / 2;

      node.children[0]->cut.index = 0;
      node.children[0]->data      = less ? value : std::move(node.data);
      node.children[1]->cut.index = 0;
      node.children[1]->data      = less ? std::move(node.data) : value;

      node.data.reset();
    }
    else {
      if (*(begin(value) + node.cut.index) < node.cut.value) {
        push(*node.children[0], value);
      }
      else {
        push(*node.children[1], value);
      }
    }
  }

  std::vector<Neighbor> search(
      const T& query,
      const std::size_t* k,
      const double* radius,
      const bool& sorted) const {
    std::vector<Neighbor> neighbors;

    if (empty()) {
      return neighbors;
    }

    if (nullptr != k) {
      neighbors.reserve(std::min(*k, size()));
    }

    std::size_t checks = 0;

    std::vector<Branch> branches;
    std::vector<double> sidedist(query.size(), 0.0);
    search(
        m_root, query, k, radius, branches, neighbors, checks, 0.0, sidedist);

    while (!branches.empty() && (!m_checks || checks < m_checks)) {
      Branch branch = std::move(branches.front());
      std::pop_heap(branches.begin(), branches.end(), BranchCompare());
      branches.pop_back();
      search(
          *branch.node,
          query,
          k,
          radius,
          branches,
          neighbors,
          checks,
          branch.dist,
          branch.sidedist);
    }

    if (sorted) {
      std::sort_heap(neighbors.begin(), neighbors.end(), NeighborCompare());
    }

    return neighbors;
  }

  void search(
      const Node& node,
      const T& query,
      const std::size_t* k,
      const double* radius,
      std::vector<Branch>& branches,
      std::vector<Neighbor>& neighbors,
      std::size_t& checks,
      double mindist,
      std::vector<double>& sidedist) const {
    using std::begin;

    if (nullptr == node.children[0] && nullptr == node.children[1]) {
      if (node.data) {
        double distance = m_metric->distance(query, *node.data);

        if (nullptr == k || neighbors.size() < *k
            || distance < neighbors.front().first) {
          if (nullptr == radius || distance < *radius) {
            if (nullptr != k && *k == neighbors.size()) {
              std::pop_heap(
                  neighbors.begin(), neighbors.end(), NeighborCompare());
              neighbors.pop_back();
            }

            neighbors.emplace_back(distance, *node.data);
            std::push_heap(
                neighbors.begin(), neighbors.end(), NeighborCompare());
          }
        }

        ++checks;
      }
    }
    else {
      double value = *(begin(query) + node.cut.index);
      double diff  = value - node.cut.value;

      std::size_t best  = diff < 0 ? 0 : 1;
      std::size_t worst = diff < 0 ? 1 : 0;

      search(
          *node.children[best],
          query,
          k,
          radius,
          branches,
          neighbors,
          checks,
          mindist,
          sidedist);

      double cutdist
          = m_metric->distance(value, node.cut.value, node.cut.index);
      double newdist = mindist - sidedist[node.cut.index] + cutdist;

      if (nullptr == k || neighbors.size() < *k
          || newdist <= neighbors.front().first) {
        if (!m_checks) {
          double dist              = sidedist[node.cut.index];
          sidedist[node.cut.index] = cutdist;
          search(
              *node.children[worst],
              query,
              k,
              radius,
              branches,
              neighbors,
              checks,
              newdist,
              sidedist);
          sidedist[node.cut.index] = dist;
        }
        else {
          std::vector<double> newsidedist(sidedist);
          newsidedist[node.cut.index] = cutdist;
          branches.emplace_back(
              newdist, std::move(newsidedist), node.children[worst].get());
          std::push_heap(branches.begin(), branches.end(), BranchCompare());
        }
      }
    }
  }

  template <typename InputIterator>
  Cut select(InputIterator first, InputIterator last) {
    using std::begin;

    std::size_t distance = std::distance(first, last);
    assert(distance > 0 || "mean expects at least one element");
    std::size_t samples = std::min(m_samples, distance);
    std::size_t dim     = size(*first);

    m_mean.resize(dim);
    std::fill(m_mean.begin(), m_mean.end(), 0.0);

    m_var.resize(dim);
    std::fill(m_var.begin(), m_var.end(), 0.0);

    for (InputIterator i = first; i < first + samples; ++i) {
      for (std::size_t j = 0; j < dim; ++j) {
        m_mean[j] += *(begin(*i) + j);
      }
    }

    for (std::size_t i = 0; i < dim; ++i) {
      m_mean[i] /= samples;
    }

    for (InputIterator i = first; i < first + samples; ++i) {
      for (std::size_t j = 0; j < dim; ++j) {
        double d = *(begin(*i) + j) - m_mean[j];
        m_var[j] += d * d;
      }
    }

    typename std::vector<double>::iterator max
        = std::max_element(m_var.begin(), m_var.end());

    Cut cut;
    cut.index = std::distance(m_var.begin(), max);
    cut.value = m_mean[cut.index];
    return cut;
  }

  std::optional<std::size_t> m_checks;

  std::vector<double> m_mean;

  ConstMetricPtr m_metric;

  Node m_root;

  std::size_t m_samples;

  std::size_t m_values;

  std::vector<double> m_var;
};

} // namespace crobot::math

