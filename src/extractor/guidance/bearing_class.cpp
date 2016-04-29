#include "extractor/guidance/bearing_class.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{
namespace
{

DiscreteAngle discretizeAngle(const double angle)
{
    BOOST_ASSERT(angle >= 0. && angle <= 360.);
    return DiscreteAngle(static_cast<std::uint8_t>(
        (angle + 0.5 * detail::discrete_angle_step_size) / detail::discrete_angle_step_size));
}

double angleFromDiscreteAngle(const DiscreteAngle angle)
{
    return static_cast<double>(angle) * detail::discrete_angle_step_size +
           0.5 * detail::discrete_angle_step_size;
}

void BearingClass::addContinuous(const double angle)
{
    auto discrete = discretizeAngle(angle);
}

}

} // namespace guidance
} // namespace extractor
} // namespace osrm
