#ifndef OSRM_EXTRACTOR_GUIDANCE_ENTRY_CLASS_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_ENTRY_CLASS_HPP_

#include <cstdint>

namespace osrm
{
namespace extractor
{
namespace guidance
{

class EntryClass
{
    using FlagBaseType = std::uint32_t;
  public:
    // we are hiding the access to the flags behind a protection wall, to make sure the bit logic
    // isn't tempered with. zero based indexing
    void activate(std::uint32_t index);
  private:
    // given a list of possible discrete angles, the available angles flag indicates the presence of
    // a given turn at the intersection
    FlagBaseType enabled_entries_flags;
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif /* OSRM_EXTRACTOR_GUIDANCE_ENTRY_CLASS_HPP_ */
