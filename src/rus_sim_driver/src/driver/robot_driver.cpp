#include "driver/robot_driver.hpp"
#include "driver/real_driver.hpp"
#include "driver/sim_driver.hpp"

#include <array>
#include <cmath>

namespace RusRobotDriver {
    // DriverFactory
    std::unique_ptr<IRobotDriver> DriverFactory::Create(Type type, const std::string& ip)
    {
        switch (type) {
            case Sim: return std::make_unique<RusSimRobotDriver::RobotSimDriver>(ip);
            case Real: return std::make_unique<RusRealRobotDriver::RobotRealDriver>(ip);
            default: return nullptr;
        }
    }

} // namespace RusRobotDriver