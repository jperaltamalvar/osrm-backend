#ifndef OSRM_EXTRACTOR_GUIDANCE_ENTRY_CLASS_HPP_
#define OSRM_EXTRACTOR_GUIDANCE_ENTRY_CLASS_HPP_

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
class EntryClass;
} // namespace guidance
} // namespace extractor
} // namespace osrm

namespace std
{
template <> struct hash<::osrm::extractor::guidance::EntryClass>
{
    inline std::size_t operator()(const ::osrm::extractor::guidance::EntryClass &entry_class) const;
};
} // namespace std

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
    EntryClass();

    // we are hiding the access to the flags behind a protection wall, to make sure the bit logic
    // isn't tempered with. zero based indexing
    void activate(std::uint32_t index);

    // representation to present to the api
    std::string getStringRepresentation() const;

    // required for hashing
    bool operator==(const EntryClass &) const;

  private:
    // given a list of possible discrete angles, the available angles flag indicates the presence of
    // a given turn at the intersection
    FlagBaseType enabled_entries_flags;

    // allow hash access to internal representation
    friend std::size_t std::hash<EntryClass>::operator()(const EntryClass &) const;
};

} // namespace guidance
} // namespace extractor
} // namespace osrm

// make Entry Class hasbable
namespace std
{
inline size_t hash<::osrm::extractor::guidance::EntryClass>::
operator()(const ::osrm::extractor::guidance::EntryClass &entry_class) const
{
    return hash<::osrm::extractor::guidance::EntryClass::FlagBaseType>()(
        entry_class.enabled_entries_flags);
}
} // namespace std

#endif /* OSRM_EXTRACTOR_GUIDANCE_ENTRY_CLASS_HPP_ */
