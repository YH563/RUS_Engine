#include "controller/controller.hpp"
#include "controller/ctc_controller.hpp"
#include "controller/gravity_comp_controller.hpp"

namespace RusRobotDriver {

    std::unique_ptr<IController> ControllerFactory::Create(
        Type type, DynamicsFunc dynamics, double kp, double kd)
    {
        switch (type) {
        case CTC:
            return std::make_unique<CtcController>(std::move(dynamics), kp, kd);
        case GravityComp:
            return std::make_unique<GravityCompController>(std::move(dynamics));
        default:
            return nullptr;
        }
    }

}  // namespace RusRobotDriver
