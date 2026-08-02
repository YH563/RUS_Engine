#include "controller/gravity_comp_controller.hpp"

namespace RusRobotDriver {

    GravityCompController::GravityCompController(DynamicsFunc dynamics)
        : dynamics_(std::move(dynamics))
    {}

    VectorXd GravityCompController::ComputeTorque(const ControlTarget& /*target*/,
                                                  const RobotState& state)
    {
        int n = static_cast<int>(state.joint_pos.size());
        Eigen::MatrixXd M(n, n);
        VectorXd bias(n);
        VectorXd qd_zero = VectorXd::Zero(n);

        dynamics_(state.joint_pos, qd_zero, M, bias);
        return bias;
    }

}  // namespace RusRobotDriver