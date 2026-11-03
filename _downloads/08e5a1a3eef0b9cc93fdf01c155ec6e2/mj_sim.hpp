#pragma once

#include <memory>
#include <mutex>
#include <unordered_map>

#include <mujoco/mujoco.h>

#include "crobot/math/math.hpp"
#include "crobot/simulation/marker.hpp"
#include "crobot/simulation/mj_rigid_body.hpp"
#include "crobot/simulation/mj_rigid_body_tree.hpp"
#include "crobot/simulation/sim_contact.hpp"
#include "crobot/simulation/simulate.hpp"

namespace crobot::simulation {

class MJSim {
public:
  /**
   * @brief Construct a new MJSim object
   *
   * @param filename The filename of the Mujoco model.
   */
  explicit MJSim(const std::string& filename);

  /**
   * @brief Destroy the MJSim object
   *
   */
  virtual ~MJSim();

  /**
   * @brief Reload the Mujoco model.
   *
   * @param filename The filename of the Mujoco model.
   * @return true If the model is reloaded successfully.
   * @return false If the model is not reloaded successfully.
   */
  bool reload_model(const std::string& filename);

  /**
   * @brief Run the simulation.
   */
  void run(bool view = true);

  /**
   * @brief Show current simulation state for a given time in milliseconds.
   */
  void show();

  /**
   * @brief Show current simulation state for a given time in milliseconds in a
   * detached thread.
   */
  void show_detached();

  /**
   * @brief Add marker to the simulation scene.
   *
   * @param markers MarkerManager that contains all the markers to be rendered.
   */
  virtual void visualize(MarkerManager& markers);

  /**
   * @brief The control callback function.
   *
   * @details The user control callback would be expected to set some of the
   * below fields to non-zero:
   *
   *  (data.ctrl, data.qfrc_applied, data.xfrc_applied)
   *
   * The values written in these vectors can depend on position, velocity and
   * all other quantities derived from them, but cannot depend on contact forces
   * and other quantities that are computed after the control is specified. If
   * the callback accesses the latter fields, their values do not correspond to
   * the current time step.
   */
  virtual void control();

  /**
   * @brief Get the RigidBodyTree from the simulation.
   *
   * @param name The name of the rigid body tree.
   * @return ConstMJRBTreePtr The rigid body tree.
   */
  ConstMJRBTreePtr get_rbtree(const std::string& name) const;

  /**
   * @brief Get the RigidBodyTree from the simulation.
   *
   * @return ConstMJRBTreePtr The rigid body tree.
   */
  ConstMJRBTreePtr get_rbtree() const;

  /**
   * @brief Register a contact point between two geometry objects.
   *
   * @param geom1 Name of the first geometry object.
   * @param geom2 Name of the second geometry object.
   * @return simulation::ConstContactPtr The contact point.
   */
  simulation::ConstContactPtr add_contact(
      const std::string& geom1, const std::string& geom2);

public:
  /// MuJoCo data structure.
  mjData* m_data;

  /// MuJoCo model.
  mjModel* m_model;

private:
  void update_contact_info();

  void add_markers_to_viewer();

  static void physics_loop(
      MJSim* mj_sim,
      ::mujoco::Simulate* sim,
      mjModel* m,
      mjData* d,
      mjtNum* ctrlnoise);

  /**
   * @brief Simulate the model.
   *
   * @param sim Pointer to a simulation object.
   */
  static void simulate(MJSim* sim);

  /**
   * @brief Visualize the model.
   *
   * @param sim Pointer to a simulation object.
   */
  static void view(MJSim* sim);

  std::string m_filename;

  // control noise variables
  mjtNum* m_ctrl_noise = nullptr;

  using hash_pair = struct hash_pair_
  {
    size_t operator()(const std::pair<int, int>& v) const {
      size_t h = (size_t(v.first) << 32) + size_t(v.second);
      h *= 1231231557ull; // "random" uneven integer
      h ^= (h >> 32);
      return h;
    }
  };
  std::unordered_map<std::pair<int, int>, simulation::ContactPtr, hash_pair>
      m_contacts;

  std::unique_ptr<mujoco::Simulate> m_viewer;
  std::unique_ptr<mjvCamera> m_mjv_cam;
  std::unique_ptr<mjvOption> m_mjv_opt;
  std::unique_ptr<mjvPerturb> m_mjv_pert;
};

} // namespace crobot::simulation

