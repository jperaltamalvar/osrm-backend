#include "extractor/guidance/entry_class.hpp"

#include <sstream>
#include <bitset>
#include <iostream>

#include <boost/assert.hpp>

namespace osrm
{
namespace extractor
{
namespace guidance
{

EntryClass::EntryClass() : enabled_entries_flags(0) {}

void EntryClass::activate(std::uint32_t index) {
    BOOST_ASSERT( index < 8 * sizeof(FlagBaseType));
    enabled_entries_flags |= (1 << index);
}

std::string EntryClass::getStringRepresentation() const
{
    std::ostringstream oss;
    oss << std::bitset<32>(enabled_entries_flags);
    return oss.str();
}

bool EntryClass::operator==(const EntryClass &other) const
{
    return enabled_entries_flags == other.enabled_entries_flags;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
