#include "extractor/guidance/bearing_class.hpp"
#include "extractor/guidance/discrete_angle.hpp"

#include <boost/assert.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{
namespace
{

static_assert(
    360 / BearingClass::discrete_angle_step_size <= 8 * sizeof(BearingClass::FlagBaseType),
    "The number of expressable bearings does not fit into the datatype used for storage.");

DiscreteAngle discretizeAngle(double angle)
{
    BOOST_ASSERT(angle >= 0. && angle <= 360.);
    // shift angle by half the step size to have the class be located around the center
    angle = (angle + 0.5 * BearingClass::discrete_angle_step_size);
    if (angle > 360)
        angle -= 360;

    return DiscreteAngle(angle / BearingClass::discrete_angle_step_size);
}

double angleFromDiscreteAngle(const DiscreteAngle angle)
{
    return static_cast<double>(angle) * BearingClass::discrete_angle_step_size +
           0.5 * BearingClass::discrete_angle_step_size;
}
}

BearingClass::BearingClass() : available_angles_flag(0) {}

bool BearingClass::operator==(const BearingClass &other) const
{
    return other.available_angles_flag == available_angles_flag;
}

void BearingClass::addContinuous(const double angle)
{
    auto discrete = discretizeAngle(angle);
    available_angles_flag |= (1 << discrete);
}

std::string BearingClass::getStringRepresentation() const
{
    std::string result = "Angles:";
    for (std::size_t bit_index = 0; bit_index < 8 * sizeof(BearingClass::FlagBaseType); ++bit_index)
    {
        if (available_angles_flag & (1 << bit_index))
        {
            result += " " + std::to_string(angleFromDiscreteAngle(DiscreteAngle(bit_index)));
        }
    }
    return result;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
