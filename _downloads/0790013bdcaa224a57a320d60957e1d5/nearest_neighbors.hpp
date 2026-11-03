#pragma once

#include <vector>

#include "crobot/math/matrix.hpp"

namespace crobot::math {

template <typename T>
class NearestNeighbors {
public:
  /// Neighbor is a pair of distance and vertex.
  typedef std::pair<double, T> Neighbor;

  NearestNeighbors() = default;

  virtual ~NearestNeighbors() = default;

  virtual void clear() = 0;

  virtual bool empty() const = 0;

  virtual std::vector<Neighbor> nearest(
      const T& query, std::size_t k, bool sorted = true) const = 0;

  virtual void push(const T& value) = 0;

  virtual std::vector<Neighbor> radius(
      const T& query, double radius, bool sorted = true) const = 0;

  virtual std::size_t size() const = 0;

protected:
private:
};
} // namespace crobot::math

