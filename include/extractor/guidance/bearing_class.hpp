#ifndef OSRM_EXTRACTOR_GUIDANCE_BEARING_CLASS_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_BEARING_CLASS_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

namespace osrm
{
namespace extractor
{
namespace guidance
{
class BearingClass;
} // namespace guidance
} // namespace extractor
} // namespace osrm

namespace std
{
template <> struct hash<::osrm::extractor::guidance::BearingClass>
{
    inline std::size_t
    operator()(const ::osrm::extractor::guidance::BearingClass &bearing_class) const;
};
} // namespace std

namespace osrm
{
namespace extractor
{
namespace guidance
{

class BearingClass
{
  public:
    using FlagBaseType = std::uint32_t;
    const static constexpr double discrete_angle_step_size = 360. / 24;

    BearingClass();

    // add a continuous angle to the
    void addContinuous(const double bearing);

    // representation to provide to the API
    std::string getStringRepresentation() const;

    // hashing
    bool operator==(const BearingClass &other) const;
    // we are hiding the access to the flags behind a protection wall, to make sure the bit logic
    // isn't tempered with
  private:
    // given a list of possible discrete angles, the available angles flag indicates the presence of
    // a given turn at the intersection
    FlagBaseType available_angles_flag;

    // allow hash access to internal representation
    friend std::size_t std::hash<BearingClass>::operator()(const BearingClass &) const;
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

// make Bearing Class hasbable
namespace std
{
inline size_t hash<::osrm::extractor::guidance::BearingClass>::
operator()(const ::osrm::extractor::guidance::BearingClass &bearing_class) const
{
    return hash<::osrm::extractor::guidance::BearingClass::FlagBaseType>()(
        bearing_class.available_angles_flag);
}
} // namespace std

#endif /* OSRM_EXTRACTOR_GUIDANCE_BEARING_CLASS_HPP_ */
