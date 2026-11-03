#include "crobot/simulation/mj_rigid_body.hpp"

#include <mujoco/mujoco.h>

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"
#include "crobot/simulation/mj_rigid_body_tree.hpp"

using namespace crobot;
using namespace simulation;

namespace {

//==============================================================================
std::string make_body_name(const mjModel* model, int bid) {
  auto body_name = mj_id2name(model, mjOBJ_BODY, bid);
  if (body_name) {
    return std::string(body_name);
  }
  else {
    return "body_" + std::to_string(bid);
  }
}

} // namespace

//==============================================================================
MJRigidBodyPtr MJRigidBody::MakeShared(
    const mjModel* model, int id, MJJointPtr joint) {
  return std::make_shared<MJRigidBody>(model, id, joint);
}

//==============================================================================
MJRigidBodyPtr MJRigidBody::MakeSharedDummy(
    const mjModel* model, const std::string& name, MJJointPtr joint) {
  return std::make_shared<MJRigidBody>(model, name, joint);
}

//==============================================================================
MJRigidBody::MJRigidBody(const mjModel* model, int bid_mj, MJJointPtr joint)
  : kdl::RigidBody(make_body_name(model, bid_mj), joint),
    m_model(model),
    m_mjid(bid_mj) {
  CROBOT_ASSERT_ERROR(
      m_model != nullptr, "Invalid Mujoco model. (model == nullptr)");
  CROBOT_ASSERT_ERROR(
      0 <= m_mjid && m_mjid < m_model->nbody,
      "Invalid body id \"%d\".",
      m_mjid);

  // body_ipos;          // local position of center of mass         (nbody x 3)
  // body_iquat;         // local orientation of inertia ellipsoid   (nbody x 4)
  // body_mass;          // mass                                     (nbody x 1)
  // body_subtreemass;   // mass of subtree starting at this body    (nbody x 1)
  // body_inertia;       // diagonal inertia in ipos/iquat frame     (nbody x 3)
  // body_invweight0;    // mean inv inert in qpos0 (trn, rot)       (nbody x 2)

  double mass = m_model->body_mass[m_mjid];
  math::Matrix3d Ii
      = math::Vector3d::Map(m_model->body_inertia + m_mjid * 3).asDiagonal();
  math::Quaternion q = math::Quaternion(
      m_model->body_iquat[m_mjid * 4 + 0],
      m_model->body_iquat[m_mjid * 4 + 1],
      m_model->body_iquat[m_mjid * 4 + 2],
      m_model->body_iquat[m_mjid * 4 + 3]);
  math::Matrix3d R  = q.as_rotm();
  math::Vector3d p  = math::Vector3d::Map(m_model->body_ipos + m_mjid * 3);
  math::Matrix3d Io = transform_inertia_com2target(Ii, mass, R, p);

  set_inertia(math::Vector6d(
      Io(0, 0), Io(1, 1), Io(2, 2), Io(1, 2), Io(0, 2), Io(0, 1)));
  set_mass(mass);
  set_com(p);

  // Add geoms
  for (int i = 0; i < m_model->body_geomnum[bid_mj]; i++) {
    int geomid       = m_model->body_geomadr[bid_mj] + i;
    int geom_type    = m_model->geom_type[geomid];
    int geom_contype = m_model->geom_contype[geomid];
    if (geom_contype == 0) {
      // skip this geometry as it is used for visualization only
      continue;
    }

    const char* geom_name_ptr = mj_id2name(m_model, mjOBJ_GEOM, geomid);
    std::string geom_name(geom_name_ptr == nullptr ? "" : geom_name_ptr);

    math::Vector3d geom_pos
        = math::Vector3d::Map(m_model->geom_pos + 3 * geomid);
    math::Quaternion geom_quat = math::Quaternion(
        m_model->geom_quat[4 * geomid + 0],
        m_model->geom_quat[4 * geomid + 1],
        m_model->geom_quat[4 * geomid + 2],
        m_model->geom_quat[4 * geomid + 3]);
    math::Matrix4d tform    = geom_quat.as_tform();
    tform.block<3, 1>(0, 3) = geom_pos;
    const double* geom_size = m_model->geom_size + 3 * geomid;

    if (mjGEOM_SPHERE == geom_type) {
      double radius = geom_size[0];
      add_geom_sphere(geom_name, radius, tform);
    }
    else if (mjGEOM_CAPSULE == geom_type) {
      double radius = geom_size[0];
      double length = 2. * geom_size[1];
      add_geom_capsule(geom_name, radius, length, tform);
    }
    else if (mjGEOM_ELLIPSOID == geom_type) {
      double ra = geom_size[0];
      double rb = geom_size[1];
      double rc = geom_size[2];
      add_geom_ellipsoid(geom_name, ra, rb, rc, tform);
    }
    else if (mjGEOM_CYLINDER == geom_type) {
      double radius = geom_size[0];
      double length = 2. * geom_size[1];
      add_geom_cylinder(geom_name, radius, length, tform);
    }
    else if (mjGEOM_BOX == geom_type) {
      double lx = 2. * geom_size[0];
      double ly = 2. * geom_size[1];
      double lz = 2. * geom_size[2];
      add_geom_box(geom_name, lx, ly, lz, tform);
    }
    else if (mjGEOM_MESH == geom_type) {
      int mesh_id = m_model->geom_dataid[geomid];

      // vertices
      int mesh_vertnum = m_model->mesh_vertnum[mesh_id];
      int mesh_vertadr = m_model->mesh_vertadr[mesh_id];
      math::Points vs(mesh_vertnum, 3);
      for (int j = 0; j < mesh_vertnum; ++j) {
        vs(j, 0) = model->mesh_vert[3 * (mesh_vertadr + j) + 0];
        vs(j, 1) = model->mesh_vert[3 * (mesh_vertadr + j) + 1];
        vs(j, 2) = model->mesh_vert[3 * (mesh_vertadr + j) + 2];
      }

      // triangles
      int mesh_facenum = model->mesh_facenum[mesh_id];
      int mesh_faceadr = model->mesh_faceadr[mesh_id];
      math::Triangles ts(mesh_facenum, 3);
      for (int j = 0; j < mesh_facenum; ++j) {
        int v1   = model->mesh_face[3 * (mesh_faceadr + j) + 0];
        int v2   = model->mesh_face[3 * (mesh_faceadr + j) + 1];
        int v3   = model->mesh_face[3 * (mesh_faceadr + j) + 2];
        ts(j, 0) = v1;
        ts(j, 1) = v2;
        ts(j, 2) = v3;
      }
      add_geom_mesh(geom_name, vs, ts, tform);
    }
  }
}

//==============================================================================
MJRigidBody::MJRigidBody(
    const mjModel* model, const std::string& name, MJJointPtr joint)
  : kdl::RigidBody(name, joint), m_model(model), m_mjid(-1) {
  set_mass(0);
  set_inertia(math::Vector6d(math::Vector6d::Zero()));
  set_com(math::Vector3d::Zero());
}

//==============================================================================
bool MJRigidBody::is_dummy_body() const {
  return m_mjid == -1;
}

//==============================================================================
MJSite::ConstPtr MJRigidBody::get_site(const std::string& name) const {
  return std::dynamic_pointer_cast<const MJSite>(
      kdl::RigidBody::get_site(name));
}

//==============================================================================
void MJRigidBody::add_site(MJSite::ConstPtr site) {
  CROBOT_ASSERT_ERROR(
      get_site(site->get_name()) == nullptr,
      "Site with name '%s' already exists.",
      site->get_name().c_str());
  m_sites.push_back(site);
}

//==============================================================================
void MJRigidBody::add_site(
    const std::string& site_name,
    kdl::Site::Type type,
    const math::Vector3d& bpos,
    const math::Quaternion& quat,
    const math::Vector3d& size) {
  CROBOT_ASSERT_ERROR(
      get_site(site_name) == nullptr,
      "Site with name '%s' already exists.",
      site_name.c_str());
  m_sites.push_back(std::make_shared<MJSite>(
      site_name,
      type,
      std::dynamic_pointer_cast<MJRigidBody>(shared_from_this()),
      bpos,
      quat,
      size));
}

//==============================================================================
math::Vector3d MJRigidBody::get_xpos() const {
  mjData* data = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree)->m_data;
  return Eigen::Map<math::Vector3d>(data->xpos + m_mjid * 3);
}

//==============================================================================
math::Vector3d MJRigidBody::get_xpos(const math::Vector3d& bpos) const {
  mjData* data = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree)->m_data;
  math::Matrix3d xmat = Eigen::Map<math::Matrix3d>(data->xmat + m_mjid * 9);
  math::Vector3d xpos = Eigen::Map<math::Vector3d>(data->xpos + m_mjid * 3);
  return xpos + xmat * bpos;
}

//==============================================================================
math::Quaternion MJRigidBody::get_xquat() const {
  mjData* data = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree)->m_data;
  return math::Quaternion(
      data->xquat[m_mjid * 4 + 0],
      data->xquat[m_mjid * 4 + 1],
      data->xquat[m_mjid * 4 + 2],
      data->xquat[m_mjid * 4 + 3]);
}

//==============================================================================
math::Matrix3d MJRigidBody::get_xmat() const {
  mjData* data = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree)->m_data;
  return Eigen::Map<math::Matrix3d>(data->xmat + m_mjid * 9);
}

//==============================================================================
math::Matrix3d MJRigidBody::get_xmat(const math::Matrix3d& rotm) const {
  return get_xmat() * rotm;
}

//==============================================================================
math::Matrix4d MJRigidBody::get_xform() const {
  math::Matrix4d tform    = get_xmat().rotm2tform();
  tform.block<3, 1>(0, 3) = get_xpos();
  return tform;
}

//==============================================================================
math::Jacobian MJRigidBody::get_jacb() const {
  ConstMJRBTreePtr rbtree = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree);
  return rbtree->get_body_jacb_mj(m_mjid);
}

//==============================================================================
math::Jacobian MJRigidBody::get_jacb_at_xpos(const math::Vector3d& xpos) const {
  ConstMJRBTreePtr rbtree = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree);
  return rbtree->get_body_jacb_at_xpos(m_mjid, xpos);
}

//==============================================================================
math::LinearJacobian MJRigidBody::get_jacp_at_xpos(
    const math::Vector3d& xpos) const {
  ConstMJRBTreePtr rbtree = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree);
  return rbtree->get_body_jacp_at_xpos_mj(m_mjid, xpos);
}

//==============================================================================
math::Vector6d MJRigidBody::get_xvel_mj() const {
  ConstMJRBTreePtr rbtree = std::dynamic_pointer_cast<const MJRBTree>(m_rbtree);
  math::Jacobian jacb     = rbtree->get_body_jacb_mj(m_mjid);
  math::VectorXd qvel     = rbtree->get_qvel_mj();
  return jacb * qvel;
}
