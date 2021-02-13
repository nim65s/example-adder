#pragma once

namespace gepetto {
  namespace example {

    template <typename Scalar>
      struct DifferentialActionDataFreeFwdDynamicsExtForcesTpl;

    template <typename Scalar>
      class DifferentialActionModelFreeFwdDynamicsExtForcesTpl;

    typedef DifferentialActionDataFreeFwdDynamicsExtForcesTpl<double> DifferentialActionDataFreeFwdDynamicsExtForces;
    typedef DifferentialActionModelFreeFwdDynamicsExtForcesTpl<double> DifferentialActionModelFreeFwdDynamicsExtForces;
    typedef pinocchio::ForceTpl<double> Force;

  }
}
