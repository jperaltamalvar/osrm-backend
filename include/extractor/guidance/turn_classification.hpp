#ifndef OSRM_GUIDANCE_TURN_CLASSIFICATION_HPP_
#define OSRM_GUIDANCE_TURN_CLASSIFICATION_HPP_

#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/toolkit.hpp"
#include "extractor/guidance/entry_class.hpp"
#include "extractor/guidance/bearing_class.hpp"

#include "util/coordinate.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "extractor/query_node.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>
#include <utility>

namespace osrm
{
namespace extractor
{
namespace guidance
{

struct TurnPossibility
{
    TurnPossibility(bool entry_allowed, DiscreteAngle bearing)
        : entry_allowed(entry_allowed), bearing(std::move(bearing))
    {
    }

    TurnPossibility() : entry_allowed(false), bearing(0) {}

    bool entry_allowed;
    DiscreteAngle bearing;
};

struct compareTurnPossibility
{
    // check lexicographi order of turn possibilities
    inline bool operator()(const std::vector<TurnPossibility> &left,
                    const std::vector<TurnPossibility> &right) const
    {
        for (std::size_t index = 0; index < std::min(left.size(), right.size()); ++index)
        {
            if (left[index].bearing < right[index].bearing)
                return true;
            if (left[index].bearing > right[index].bearing)
                return false;
            /*
            if (left[index].entry_allowed && !right[index].entry_allowed)
                return true;
            if (!left[index].entry_allowed && right[index].entry_allowed)
                return false;
            */
        }
        return left.size() < right.size();
    }
};

struct compareEntryClass
{
    // check lexicographi order of turn possibilities
    inline bool operator()(const std::vector<TurnPossibility> &left,
                    const std::vector<TurnPossibility> &right) const
    {
        for (std::size_t index = 0; index < std::min(left.size(), right.size()); ++index)
        {
            /*
            if (left[index].bearing < right[index].bearing)
                return true;
            if (left[index].bearing > right[index].bearing)
                return false;
            */
            if (left[index].entry_allowed && !right[index].entry_allowed)
                return true;
            if (!left[index].entry_allowed && right[index].entry_allowed)
                return false;
        }
        return left.size() < right.size();
    }

};

std::pair<EntryClass,BearingClass>
classifyIntersection(NodeID nid,
                     const Intersection &intersection,
                     const util::NodeBasedDynamicGraph &node_based_graph,
                     const extractor::CompressedEdgeContainer &compressed_geometries,
                     const std::vector<extractor::QueryNode> &query_nodes);

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_GUIDANCE_TURN_CLASSIFICATION_HPP_
