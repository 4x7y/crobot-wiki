#pragma once

#include <algorithm>
#include <optional>
#include <vector>

#include "crobot/math/matrix.hpp"
#include "crobot/math/metric.hpp"
#include "crobot/math/nearest_neighbors.hpp"
#include "crobot/math/state_space.hpp"

namespace crobot::math {

/**
 * Linear nearest neighbor search.
 */
template <typename T>
class LinearNearestNeighbors : public NearestNeighbors<T> {
public:
  using Neighbor       = std::pair<double, T>;
  using ConstMetricPtr = std::shared_ptr<const Metric<T>>;

  explicit LinearNearestNeighbors(ConstMetricPtr metric)
    : NearestNeighbors<T>(), m_container(), m_metric(metric) {
  }

  ~LinearNearestNeighbors() = default;

  void clear() override {
    m_container.clear();
  }

  bool empty() const override {
    return m_container.empty();
  }

  std::vector<Neighbor> nearest(
      const T& query, std::size_t k, bool sorted = true) const override {
    return search(query, std::optional<std::size_t>(k), std::nullopt, sorted);
  }

  T& operator[](const std::size_t& i) {
    return m_container[i];
  }

  const T& operator[](const std::size_t& i) const {
    return m_container[i];
  }

  void push(const T& value) {
    m_container.push_back(value);
  }

  std::vector<Neighbor> radius(
      const T& query, double radius, bool sorted = true) const {
    return search(query, std::nullopt, radius, sorted);
  }

  void reserve(const std::size_t& capacity) {
    m_container.reserve(capacity);
  }

  std::size_t size() const {
    return m_container.size();
  }

  void swap(LinearNearestNeighbors& other) {
    std::swap(m_container, other.m_container);
    std::swap(m_metric, other.m_metric);
  }

  friend void swap(LinearNearestNeighbors& lhs, LinearNearestNeighbors& rhs) {
    lhs.swap(rhs);
  }

private:
  std::vector<Neighbor> search(
      const T& query,
      std::optional<std::size_t> k,
      std::optional<double> radius,
      bool sorted) const {
    std::vector<Neighbor> neighbors;
    if (m_container.empty()) {
      return neighbors;
    }
    if (k.has_value()) {
      neighbors.reserve(std::min(k.value(), size()));
    }

    std::function<bool(const Neighbor& lhs, const Neighbor& rhs)> comparator
        = [](const Neighbor& lhs, const Neighbor& rhs) {
            return lhs.first < rhs.first;
          };

    for (std::size_t i = 0; i < m_container.size(); ++i) {
      double distance = m_metric->distance(query, m_container[i]);

      if (!k.has_value() ||               //
          neighbors.size() < k.value() || //
          distance < neighbors.front().first) {

        // only select the neighbors within the radius if the radius is set
        if (!radius.has_value() || distance < radius.value()) {

          // pop out the neightbor with the largest distance if the number of
          // neighbors is greater than k
          if (k.has_value() && k.value() == neighbors.size()) {

            // moves the largest to the end
            std::pop_heap(neighbors.begin(), neighbors.end(), comparator);
            // removes the largest element
            neighbors.pop_back();
          }

          neighbors.emplace_back(distance, m_container[i]);
          std::push_heap(neighbors.begin(), neighbors.end(), comparator);
        }
      }
    }

    if (sorted) {
      std::sort_heap(neighbors.begin(), neighbors.end(), comparator);
    }

    return neighbors;
  }

  std::vector<T> m_container;

  ConstMetricPtr m_metric;
};

} // namespace crobot::math

