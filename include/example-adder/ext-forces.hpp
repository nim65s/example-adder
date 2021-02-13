///////////////////////////////////////////////////////////////////////////////
// Writting a DAM similar to free-fwddyn but taking into account exterior forces applying on the model
//
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

// can I put whatever I want here?
#ifndef __example_adder_ext_forces__
#define __example_adder_ext_forces__

#include <stdexcept>

#ifdef PINOCCHIO_WITH_CPPAD_SUPPORT  // TODO(cmastalli): Removed after merging Pinocchio v.2.4.8
#include <pinocchio/codegen/cppadcg.hpp>
#endif

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/actuation-base.hpp"
#include "crocoddyl/multibody/data/multibody.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"
#include "crocoddyl/core/utils/exception.hpp"

#include "example-adder/fwd.hpp"

namespace gepetto {
namespace example {

using namespace crocoddyl;

template <typename _Scalar>
class DifferentialActionModelFreeFwdDynamicsExtForcesTpl : public DifferentialActionModelAbstractTpl<_Scalar> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef DifferentialActionModelAbstractTpl<Scalar> Base;
  typedef DifferentialActionDataFreeFwdDynamicsExtForcesTpl<Scalar> Data;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef CostModelSumTpl<Scalar> CostModelSum;
  typedef StateMultibodyTpl<Scalar> StateMultibody;
  typedef ActuationModelAbstractTpl<Scalar> ActuationModelAbstract;
  typedef DifferentialActionDataAbstractTpl<Scalar> DifferentialActionDataAbstract;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  typedef PINOCCHIO_ALIGNED_STD_VECTOR(Force) ForceAlignedVector; // is typename needed? probably not

  DifferentialActionModelFreeFwdDynamicsExtForcesTpl(boost::shared_ptr<StateMultibody> state,
                                            boost::shared_ptr<ActuationModelAbstract> actuation,
                                            boost::shared_ptr<CostModelSum> costs,
                                            const ForceAlignedVector& extforces);
  virtual ~DifferentialActionModelFreeFwdDynamicsExtForcesTpl();

  virtual void calc(const boost::shared_ptr<DifferentialActionDataAbstract>& data, const Eigen::Ref<const VectorXs>& x,
                    const Eigen::Ref<const VectorXs>& u);
  virtual void calcDiff(const boost::shared_ptr<DifferentialActionDataAbstract>& data,
                        const Eigen::Ref<const VectorXs>& x, const Eigen::Ref<const VectorXs>& u);
  virtual boost::shared_ptr<DifferentialActionDataAbstract> createData();
  virtual bool checkData(const boost::shared_ptr<DifferentialActionDataAbstract>& data);

  virtual void quasiStatic(const boost::shared_ptr<DifferentialActionDataAbstract>& data, Eigen::Ref<VectorXs> u,
                           const Eigen::Ref<const VectorXs>& x, const std::size_t& maxiter = 100,
                           const Scalar& tol = Scalar(1e-9));

  const boost::shared_ptr<ActuationModelAbstract>& get_actuation() const;
  const boost::shared_ptr<CostModelSum>& get_costs() const;
  pinocchio::ModelTpl<Scalar>& get_pinocchio() const;
  const VectorXs& get_armature() const;
  void set_armature(const VectorXs& armature);

 protected:
  using Base::has_control_limits_;  //!< Indicates whether any of the control limits
  using Base::nr_;                  //!< Dimension of the cost residual
  using Base::nu_;                  //!< Control dimension
  using Base::state_;               //!< Model of the state
  using Base::u_lb_;                //!< Lower control limits
  using Base::u_ub_;                //!< Upper control limits
  using Base::unone_;               //!< Neutral state

 private:
  boost::shared_ptr<ActuationModelAbstract> actuation_;
  boost::shared_ptr<CostModelSum> costs_;
  pinocchio::ModelTpl<Scalar>& pinocchio_;
  bool with_armature_;
  VectorXs armature_;
  ForceAlignedVector extforces_; // does is have to be a shared pointer?
};

template <typename _Scalar>
struct DifferentialActionDataFreeFwdDynamicsExtForcesTpl : public DifferentialActionDataAbstractTpl<_Scalar> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef _Scalar Scalar;
  typedef MathBaseTpl<Scalar> MathBase;
  typedef DifferentialActionDataAbstractTpl<Scalar> Base;
  typedef typename MathBase::VectorXs VectorXs;
  typedef typename MathBase::MatrixXs MatrixXs;

  template <template <typename Scalar> class Model>
  explicit DifferentialActionDataFreeFwdDynamicsExtForcesTpl(Model<Scalar>* const model)
      : Base(model),
        pinocchio(pinocchio::DataTpl<Scalar>(model->get_pinocchio())),
        multibody(&pinocchio, model->get_actuation()->createData()),
        costs(model->get_costs()->createData(&multibody)),
        Minv(model->get_state()->get_nv(), model->get_state()->get_nv()),
        u_drift(model->get_nu()),
        dtau_dx(model->get_nu(), model->get_state()->get_ndx()),
        tmp_xstatic(model->get_state()->get_nx()) {
    costs->shareMemory(this);
    Minv.setZero();
    u_drift.setZero();
    dtau_dx.setZero();
    tmp_xstatic.setZero();
  }

  pinocchio::DataTpl<Scalar> pinocchio;
  DataCollectorActMultibodyTpl<Scalar> multibody;
  boost::shared_ptr<CostDataSumTpl<Scalar> > costs;
  MatrixXs Minv;
  VectorXs u_drift;
  MatrixXs dtau_dx;
  VectorXs tmp_xstatic;

  using Base::cost;
  using Base::Fu;
  using Base::Fx;
  using Base::Lu;
  using Base::Luu;
  using Base::Lx;
  using Base::Lxu;
  using Base::Lxx;
  using Base::r;
  using Base::xout;
};

}  // namespace example
}  // namespace gepetto

#include "example-adder/ext-forces.hxx"

#endif
