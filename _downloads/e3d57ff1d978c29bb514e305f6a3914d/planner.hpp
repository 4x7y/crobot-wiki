#pragma once

#include <chrono>
#include <memory>
#include <string>

#include "crobot/math/matrix.hpp"
#include "crobot/math/state_space.hpp"

namespace crobot::planning {

class Viewer;

class Planner {
public:
  using ConstStateSpacePtr
      = std::shared_ptr<const math::StateSpace<math::VectorXd>>;

  Planner(ConstStateSpacePtr state_space);

  virtual ~Planner();

  Planner::ConstStateSpacePtr get_state_space() const;

  virtual std::string get_algorithm_name() const = 0;

  /**
   * Get solution path.
   *
   * @pre solve()
   */
  virtual std::vector<math::VectorXd> get_path() = 0;

  /**
   * Reset planner.
   */
  virtual void reset() = 0;

  void set_duration(const std::chrono::steady_clock::duration& duration);

  std::chrono::steady_clock::duration get_duration() const;

  void set_viewer(std::shared_ptr<Viewer> viewer);

  std::shared_ptr<Viewer> get_viewer() const;

  /**
   * Find collision free path.
   */
  virtual bool plan(const math::VectorXd& start, const math::VectorXd& goal)
      = 0;

protected:
  /// Upper bound for search
  std::chrono::steady_clock::duration m_duration;

  /// State space
  ConstStateSpacePtr m_state_space;

  /// Viewer
  std::shared_ptr<Viewer> m_viewer;

  /// Time
  std::chrono::steady_clock::time_point m_time;

private:
};

} // namespace crobot::planning

