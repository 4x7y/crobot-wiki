// #ifndef CROBOT_PLANNING_MODEL_HPP
// #define CROBOT_PLANNING_MODEL_HPP

// #include "crobot/kdl/kdl.hpp"
// #include "crobot/math/matrix.hpp"

// namespace crobot::planning {

// class Model {
// public:
//   Model();

//   virtual ~Model();

//   virtual bool areColliding(const std::size_t& i, const std::size_t& j)
//   const;

//   virtual void clamp(math::VectorXd& q) const;

//   virtual double distance(
//       const math::VectorXd& q1, const math::VectorXd& q2) const;

//   virtual void forwardForce(const math::VectorXd& tau, math::VectorXd& f)
//   const;

//   virtual const math::Matrix4d& forwardPosition(std::size_t i = 0) const;

//   virtual void forwardVelocity(
//       const math::VectorXd& qdot, math::VectorXd& xdot) const;

//   virtual math::VectorXd generatePositionGaussian(
//       const math::VectorXd& rand,
//       const math::VectorXd& mean,
//       const math::VectorXd& sigma) const;

//   virtual math::VectorXd generatePositionUniform(
//       const math::VectorXd& rand) const;

//   virtual math::VectorXd generatePositionUniform(
//       const math::VectorXd& rand,
//       const math::VectorXd& min,
//       const math::VectorXd& max) const;

//   virtual kdl::ConstRigidBodyPtr getBody(const std::size_t& i) const;

//   virtual std::size_t getBodies() const;

//   virtual void getBoundingBoxPoints(
//       const std::size_t& i,
//       const math::Matrix4d& frame,
//       std::vector<math::VectorXd>& p) const;

//   virtual math::VectorXd& getCenter(const std::size_t& i) const;

//   virtual int getDof() const;

//   virtual int getDofPosition() const;

//   virtual const math::Matrix4d& getFrame(std::size_t i) const;

//   virtual const math::MatrixXd& getJacobian() const;

//   virtual double getManipulabilityMeasure() const;

//   virtual std::string getManufacturer() const;

//   virtual math::VectorXd getMaximum() const;

//   virtual math::VectorXd getMinimum() const;

//   virtual std::string getName() const;

//   virtual std::size_t getOperationalDof() const;

//   virtual Eigen::Matrix<bool, Eigen::Dynamic, 1> getWraparounds() const;

//   virtual void inverseForce(const math::VectorXd& f, math::VectorXd& tau)
//   const;

//   virtual double inverseOfTransformedDistance(const double& d) const;

//   virtual void inverseVelocity(
//       const math::VectorXd& tdot, math::VectorXd& qdot) const;

//   virtual void interpolate(
//       const math::VectorXd& q1,
//       const math::VectorXd& q2,
//       const double& alpha,
//       math::VectorXd& q) const;

//   virtual bool isColliding(const std::size_t& i) const;

//   virtual bool isSingular() const;

//   virtual bool isValid(const math::VectorXd& q) const;

//   virtual void reset();

//   virtual void setFrame(const std::size_t& i, const math::Matrix4d& frame);

//   virtual void setMaximum(const math::VectorXd& max);

//   virtual void setMinimum(const math::VectorXd& min);

//   virtual void setPosition(const math::VectorXd& q);

//   virtual void step(
//       const math::VectorXd& q1,
//       const math::VectorXd& dq,
//       math::VectorXd& q2) const;

//   virtual double transformedDistance(const double& d) const;

//   virtual double transformedDistance(
//       const math::VectorXd& q1, const math::VectorXd& q2) const;

//   virtual double transformedDistance(
//       const double& q1, const double& q2, const std::size_t& i) const;

//   virtual void updateFrames(const bool& doUpdateModel = true);

//   virtual void updateJacobian();

//   virtual void updateJacobianInverse(
//       const double& lambda = 0, const bool& doSvd = true);

// protected:
// private:
// };

// } // namespace crobot::planning

// #endif // CROBOT_PLANNING_MODEL_HPP
