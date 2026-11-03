#pragma once

#include <memory>

#include "crobot/math/matrix.hpp"

namespace crobot::kdl {

class Frame {
public:
  using Ptr      = std::shared_ptr<Frame>;
  using ConstPtr = std::shared_ptr<const Frame>;

  virtual ~Frame() = default;

  virtual math::Vector3d get_xpos() const;

  virtual math::Matrix3d get_xmat() const;

  virtual math::Matrix4d get_xform() const = 0;

  virtual math::Vector3d get_xvelp() const = 0;

  virtual math::Vector3d get_xvelr() const = 0;

  virtual math::Vector6d get_xvel() const = 0;

  virtual math::Vector3d get_xaccp() const = 0;

  virtual math::Vector3d get_xaccr() const = 0;

  virtual math::Vector6d get_xacc() const = 0;

  virtual void set_xform(const math::Matrix4d& xform) = 0;

  virtual void translate(const math::Vector3d& tvec) = 0;

  virtual void set_tvec(const math::Vector3d& tvec) = 0;

  virtual void set_xvel(const math::VectorXd& xvel) = 0;

  virtual void set_xacc(const math::VectorXd& xacc) = 0;

  virtual void set_xwrench_ext(const math::Vector6d& xwrench_ext) = 0;

  virtual math::Vector6d get_xwrench_ext() const = 0;
};

using FramePtr      = std::shared_ptr<Frame>;
using ConstFramePtr = std::shared_ptr<const Frame>;

class WorldFrame;
using WorldFramePtr      = std::shared_ptr<WorldFrame>;
using ConstWorldFramePtr = std::shared_ptr<const WorldFrame>;

class WorldFrame : public Frame {
public:
  using Ptr      = std::shared_ptr<WorldFrame>;
  using ConstPtr = std::shared_ptr<const WorldFrame>;

  static WorldFramePtr MakeShared(
      const math::Matrix4d& xform = math::Matrix4d::Identity());

  WorldFrame(const math::Matrix4d& xform = math::Matrix4d::Identity());
  virtual ~WorldFrame() = default;

  // Documentation Inherited
  math::Matrix4d get_xform() const override;
  math::Vector3d get_xvelp() const override;
  math::Vector3d get_xvelr() const override;
  math::Vector6d get_xvel() const override;
  math::Vector3d get_xaccp() const override;
  math::Vector3d get_xaccr() const override;
  math::Vector6d get_xacc() const override;

  void set_xform(const math::Matrix4d& xform) override;
  void translate(const math::Vector3d& tvec) override;
  void set_tvec(const math::Vector3d& tvec) override;
  void set_xvel(const math::VectorXd& xvel) override;
  void set_xacc(const math::VectorXd& xacc) override;
  void set_xwrench_ext(const math::Vector6d& xwrench_ext) override;

  math::Vector6d get_xwrench_ext() const override;

private:
  math::Matrix4d m_xform;
  math::Vector6d m_xvel;
  math::Vector6d m_xacc;

  math::Vector6d m_xwrench_ext;
};

} // namespace crobot::kdl

