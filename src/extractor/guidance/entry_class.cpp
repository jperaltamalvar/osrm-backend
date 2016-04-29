#include "extractor/guidance/entry_class.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

void EntryClass::activate(std::uint32_t index) { enabled_entries_flags |= (1 << index); }

} // namespace guidance
} // namespace extractor
} // namespace osrm
