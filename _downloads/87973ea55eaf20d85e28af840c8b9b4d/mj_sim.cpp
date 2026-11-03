#include "crobot/simulation/mj_sim.hpp"

#include <chrono>
#include <mutex>
#include <thread>
#include <utility>

#include <mujoco/mujoco.h>
#include <stdio.h>
#include <string.h>

#include "crobot/common/common.hpp"
#include "crobot/common/tabulate.hpp"
#include "crobot/simulation/array_safety.hpp"
#include "crobot/simulation/glfw_adapter.hpp"
#include "crobot/simulation/mj_constants.hpp"
#include "crobot/simulation/mj_rigid_body.hpp"
#include "crobot/simulation/mj_rigid_body_tree.hpp"

using namespace crobot;
using namespace simulation;

namespace mj  = ::mujoco;
namespace mju = ::mujoco::sample_util;

// maximum mis-alignment before re-sync (simulation seconds)
const double syncMisalign = 0.1;
// fraction of refresh available for simulation
const double simRefreshFraction = 0.7;
// load error string length
const int kErrorLength = 1024;

using Seconds = std::chrono::duration<double>;

namespace {

//==============================================================================
// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
//==============================================================================
std::string getExecutableDir() {
  constexpr char kPathSep = '/';
  const char* path        = "/proc/self/exe";
  std::string realpath    = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success           = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      ssize_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success                 = true;
      }
      else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: "
                  << strerror(errno) << '\n';
        return "";
      }
      else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

//==============================================================================
// Scan for libraries in the plugin directory to load additional plugins
//==============================================================================
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }
  const std::string sep = "/";

  // try to open the ${EXECDIR}/plugin directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + "mujoco_plugin";
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}
} // namespace

//==============================================================================
MJSim::MJSim(const std::string& filename) : m_filename(filename) {
  // print version, check compatibility
  CROBOT_LOG_INFO("MuJoCo Pro version {0:.2f}", 0.01 * mj_version());
  CROBOT_ASSERT_ERROR(
      mjVERSION_HEADER == mj_version(),
      "Mujoco headers and library have different versions");

  char error[500] = "";
  m_model         = mj_loadXML(filename.c_str(), NULL, error, 500);
  CROBOT_ASSERT_ERROR(
      m_model != nullptr && !error[0], "Mujoco loadXML error: %s", error);
  m_data = mj_makeData(m_model);
  CROBOT_ASSERT_ERROR(
      m_data != nullptr, "Unable to allocate Mujoco data structure.");
  mj_forward(m_model, m_data);

  m_ctrl_noise = static_cast<mjtNum*>(malloc(sizeof(mjtNum) * m_model->nu));
  mju_zero(m_ctrl_noise, m_model->nu);

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  m_mjv_cam = std::make_unique<mjvCamera>();
  mjv_defaultCamera(m_mjv_cam.get());

  m_mjv_opt = std::make_unique<mjvOption>();
  mjv_defaultOption(m_mjv_opt.get());

  m_mjv_pert = std::make_unique<mjvPerturb>();
  mjv_defaultPerturb(m_mjv_pert.get());
}

//==============================================================================
MJSim::~MJSim() {
  mj_deleteModel(m_model);
  mj_deleteData(m_data);
  free(m_ctrl_noise);
}

//==============================================================================
void MJSim::physics_loop(
    MJSim* mj_sim,
    mj::Simulate* sim,
    mjModel* m,
    mjData* d,
    mjtNum* ctrlnoise) {
  // cpu-sim syncronization point
  std::chrono::time_point<std::chrono::steady_clock> syncCPU;
  mjtNum syncSim = 0;
  sim->load_message_clear();
  sim->load(m, d, mj_sim->m_filename.c_str());

  // run until asked to exit
  while (!sim->exitrequest.load()) {
    if (sim->run && sim->busywait) {
      std::this_thread::yield();
    }
    else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      // run only if model is present
      if (m) {
        // running
        if (sim->run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim     = d->time - syncSim;

          // inject noise
          if (sim->ctrl_noise_std) {
            // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
            mjtNum rate = mju_exp(
                -m->opt.timestep / mju_max(sim->ctrl_noise_rate, mjMINVAL));
            mjtNum scale = sim->ctrl_noise_std * mju_sqrt(1 - rate * rate);

            for (int i = 0; i < m->nu; i++) {
              // update noise
              ctrlnoise[i]
                  = rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

              // apply noise
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          // requested slow-down factor
          double slowdown = 100 / sim->percentRealTime[sim->real_time_index];

          // misalignment condition: distance from target sim time is bigger
          // than syncmisalign
          bool misaligned
              = mju_abs(Seconds(elapsedCPU).count() / slowdown - elapsedSim)
                > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0
              || syncCPU.time_since_epoch().count() == 0 || misaligned
              || sim->speed_changed) {
            // re-sync
            syncCPU            = startCPU;
            syncSim            = d->time;
            sim->speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step1(m, d);
            mj_sim->update_contact_info();
            mj_sim->control();
            mj_step2(m, d);
            stepped = true;
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured  = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction / sim->refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim) * slowdown)
                       < mj::Simulate::Clock::now() - syncCPU
                   && mj::Simulate::Clock::now() - startCPU
                          < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim->measured_slowdown
                    = std::chrono::duration<double>(elapsedCPU).count()
                      / elapsedSim;
                measured = true;
              }

              // call mj_step
              mj_step1(m, d);
              mj_sim->update_contact_info();
              mj_sim->control();
              mj_step2(m, d);
              stepped = true;

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim->add_to_history();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim->speed_changed = true;
        }
      }
    } // release std::lock_guard<std::mutex>
  }
}

//==============================================================================
void MJSim::view(MJSim* sim) {
  (void)sim;
}

//==============================================================================
void MJSim::run(bool view) {
  (void)view;

  // simulate object encapsulates the UI
  m_viewer = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      m_mjv_cam.get(),
      m_mjv_opt.get(),
      m_mjv_pert.get(),
      false);
  m_viewer->load(m_model, m_data, m_filename.c_str());

  std::thread physics_thread(
      &physics_loop, this, m_viewer.get(), m_model, m_data, m_ctrl_noise);

  // start simulation UI loop (blocking call)
  m_viewer->render_loop(
      std::bind(&MJSim::visualize, this, std::placeholders::_1));
  physics_thread.join();
}

//==============================================================================
void MJSim::show() {
  // simulate object encapsulates the UI
  m_viewer = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      m_mjv_cam.get(),
      m_mjv_opt.get(),
      m_mjv_pert.get(),
      false);

  std::thread model_loading_thread(
      [this]() { m_viewer->load(m_model, m_data, m_filename.c_str()); });
  m_viewer->render_loop(
      std::bind(&MJSim::visualize, this, std::placeholders::_1));
  model_loading_thread.join();
}

//==============================================================================
void MJSim::show_detached() {
  std::thread viewer_thread(&MJSim::show, this);
  viewer_thread.detach();
}

//==============================================================================
void MJSim::visualize(MarkerManager& /* markers */) {
  // do nothing by default
}

//==============================================================================
void MJSim::control() {
  // do nothing by default
}

//==============================================================================
ConstMJRBTreePtr MJSim::get_rbtree(const std::string& root_body_name) const {
  std::string robot_name(root_body_name);
  return MJRBTree::MakeShared(robot_name, m_model, m_data, root_body_name);
}

//==============================================================================
ConstMJRBTreePtr MJSim::get_rbtree() const {
  return MJRBTree::MakeShared(m_model, m_data);
}

//==============================================================================
simulation::ConstContactPtr MJSim::add_contact(
    const std::string& geom1, const std::string& geom2) {
  int geom1_id = mj_name2id(m_model, mjOBJ_GEOM, geom1.c_str());
  int geom2_id = mj_name2id(m_model, mjOBJ_GEOM, geom2.c_str());

  auto it = m_contacts.find(std::make_pair<>(geom1_id, geom2_id));
  if (it != m_contacts.end()) {
    CROBOT_LOG_WARN(
        "Contact (geom1={}, geom2={}) has been already added to the sim->",
        geom1_id,
        geom2_id);
    return it->second;
  }

  simulation::ContactPtr contact = simulation::SimContact::MakeShared(
      geom1_id, geom2_id, kdl::ContactModel::POINT_CONTACT_WITH_FRICTION, 1.0);
  m_contacts[std::make_pair<>(geom1_id, geom2_id)] = contact;
  return contact;
}

//==============================================================================
void MJSim::update_contact_info() {
  // reset all registered contacts
  for (const auto& [geom_ids, contact] : m_contacts) {
    (void)geom_ids; // unused
    contact->reset();
  }

  // update contact states
  for (int i = 0; i < m_data->ncon; ++i) {
    mjContact* mj_contact = &m_data->contact[i];
    int geom1_id          = mj_contact->geom1;
    int geom2_id          = mj_contact->geom2;

    // geom1, geom2
    auto it = m_contacts.find(std::make_pair<>(geom1_id, geom2_id));
    if (it == m_contacts.end()) {
      it = m_contacts.find(std::make_pair<>(geom2_id, geom1_id));
      if (it == m_contacts.end()) {
        continue;
      }
      std::swap(geom1_id, geom2_id);
    }

    // force torque in contact frame
    math::Vector6d force_torque;
    mj_contactForce(m_model, m_data, i, force_torque.data());

    // In MuJoCo, the first axis of the contact frame is the contact normal.
    // We convert it back to the stardard definition.
    math::Matrix3d frame;
    frame.col(0) = Eigen::Map<math::Vector3d>(mj_contact->frame + 3);
    frame.col(1) = Eigen::Map<math::Vector3d>(mj_contact->frame + 6);
    frame.col(2) = Eigen::Map<math::Vector3d>(mj_contact->frame);
    // The MuJoCo contact frame. We will use this frame to transform wrench
    // back to the world frame.
    math::Matrix3d frame_mj;
    frame_mj.col(0) = Eigen::Map<math::Vector3d>(mj_contact->frame);
    frame_mj.col(1) = Eigen::Map<math::Vector3d>(mj_contact->frame + 3);
    frame_mj.col(2) = Eigen::Map<math::Vector3d>(mj_contact->frame + 6);

    math::Vector3d xfrc = frame_mj * force_torque.segment(0, 3);
    math::Vector3d xtrq = frame_mj * force_torque.segment(3, 3);

    math::Vector3d xpos_geom1
        = Eigen::Map<math::Vector3d>(m_data->geom_xpos + geom1_id * 3);
    math::Vector3d xpos_geom2
        = Eigen::Map<math::Vector3d>(m_data->geom_xpos + geom2_id * 3);
    math::Vector3d xvec_g1g2 = xpos_geom2 - xpos_geom1;
    math::Vector3d xvec_normal_g1g2
        = frame_mj.col(0) * frame_mj.col(0).dot(xvec_g1g2);
    if (xfrc.dot(xvec_normal_g1g2) < 0) {
      xfrc = -xfrc;
      xtrq = -xtrq;
    }
    it->second->update(
        Eigen::Map<math::Vector3d>(mj_contact->pos), frame, xfrc, xtrq);
  }
}
