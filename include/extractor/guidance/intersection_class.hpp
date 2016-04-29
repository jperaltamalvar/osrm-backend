#ifndef OSRM_EXTRACTOR_GUIDANCE_INTERSECTION_CLASS_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_INTERSECTION_CLASS_HPP_

#include <cstdint>
#include <boost/assert.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

// An intersection class is stored with the nodes. It gives indices into the memory facades storing
// entry classes in the form of entry allowed flags and bearing classes listing the available
// bearings at the intersection
struct IntersectionClass
{
    // at most 1 024 entry classes
    const constexpr static auto ENTRY_CLASS_BITS = 10;
    // at most 4 194 304 bearing classes
    const constexpr static auto BEARING_CLASS_BITS = 22;

    std::uint32_t entry_class:ENTRY_CLASS_BITS;
    std::uint32_t bearing_class:BEARING_CLASS_BITS;

    void set( std::uint32_t new_entry_class, std::uint32_t new_bearing_class )
    {
        BOOST_ASSERT(new_entry_class < (1<<(ENTRY_CLASS_BITS-1)));
        BOOST_ASSERT(new_bearing_class < (1<<(BEARING_CLASS_BITS-1)));
        entry_class = new_entry_class;
        bearing_class = new_bearing_class;
    }
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /* OSRM_EXTRACTOR_GUIDANCE_INTERSECTION_CLASS_HPP_ */
