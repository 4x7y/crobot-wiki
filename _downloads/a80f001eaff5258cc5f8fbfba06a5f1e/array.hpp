#pragma once

#include <stdexcept>
#include <vector>

#include <assert.h>

namespace crobot::math {

/**
 * @brief A simple class for representing 2D runtime arrays.
 */
template <typename T>
class array2 {
public:
  array2()                     = default;
  array2(const array2<T>& arr) = default;
  array2& operator=(const array2& arr) = default;
  array2(array2<T>&& arr)              = default;
  array2& operator=(array2&& arr) = default;
  array2(int rows, int cols, T default_value = T()) {
    resize(rows, cols, default_value);
  }
  array2(int rows, int cols, const std::vector<T>& arr)
    : rows_(rows), cols_(cols), data_(arr) {
    if (int(arr.size()) != rows * cols) {
      throw std::runtime_error("Dimensions do not match with size of vector");
    }
  }
  void resize(int rows, int cols, T val = T()) {
    data_.resize(rows * cols, val);
    rows_ = rows;
    cols_ = cols;
  }
  void clear() {
    rows_ = cols_ = 0;
    data_.clear();
  }
  T operator()(int row, int col) const {
    assert(row < rows_ && col < cols_);
    return data_[row * cols_ + col];
  }
  T& operator()(int row, int col) {
    assert(row < rows_ && col < cols_);
    return data_[row * cols_ + col];
  }
  T operator[](int idx) const {
    assert(idx < int(data_.size()));
    return data_[idx];
  }
  T& operator[](int idx) {
    assert(idx < int(data_.size()));
    return data_[idx];
  }
  int rows() const {
    return rows_;
  }
  int cols() const {
    return cols_;
  }
  int size() const {
    return int(data_.size());
  }

private:
  int rows_, cols_;
  std::vector<T> data_;
};

} // namespace crobot::math

