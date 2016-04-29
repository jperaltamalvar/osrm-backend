#ifndef OSRM_EXTRACTOR_GUIDANCE_BEARING_CLASS_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_BEARING_CLASS_HPP_

#include <cstdint>

namespace osrm
{
namespace extractor
{
namespace guidance
{

const constexpr double discrete_angle_step_size = 360. / 24;



class BearingClass
{
    using FlagBaseType = std::uint32_t;
    static_assert(360 / discrete_angle_step_size <= 8 * sizeof(FlagBaseType),"The number of expressable bearings does not fit into the datatype used for storage.");
  public:
    void addContinuous(const double bearing);
    // we are hiding the access to the flags behind a protection wall, to make sure the bit logic
    // isn't tempered with
  private:
    // given a list of possible discrete angles, the available angles flag indicates the presence of
    // a given turn at the intersection
    FlagBaseType available_angles_flag;
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /* OSRM_EXTRACTOR_GUIDANCE_BEARING_CLASS_HPP_ */
