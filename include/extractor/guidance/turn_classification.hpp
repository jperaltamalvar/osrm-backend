#ifndef OSRM_GUIDANCE_TURN_CLASSIFICATION_HPP_
#define OSRM_GUIDANCE_TURN_CLASSIFICATION_HPP_

#include "extractor/guidance/intersection.hpp"
#include "extractor/guidance/toolkit.hpp"

#include "util/coordinate.hpp"
#include "util/node_based_graph.hpp"
#include "util/typedefs.hpp"

#include "extractor/compressed_edge_container.hpp"
#include "extractor/query_node.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

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
    bool operator()(const std::vector<TurnPossibility> &left,
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

inline std::vector<TurnPossibility>
classifyIntersection(NodeID nid,
                     const Intersection &intersection,
                     const util::NodeBasedDynamicGraph &node_based_graph,
                     const extractor::CompressedEdgeContainer &compressed_geometries,
                     const std::vector<extractor::QueryNode> &query_nodes)
{

    std::vector<TurnPossibility> turns;

    const auto node_coordinate = util::Coordinate(query_nodes[nid].lon, query_nodes[nid].lat);

    // generate a list of all turn angles between a base edge, the node and a current edge
    for (const auto &road : intersection)
    {
        const auto eid = road.turn.eid;
        const auto edge_coordinate = getRepresentativeCoordinate(
            nid, node_based_graph.GetTarget(eid), eid, false, compressed_geometries, query_nodes);

        double bearing = util::coordinate_calculation::bearing(node_coordinate, edge_coordinate);
        turns.emplace_back(road.entry_allowed, discretizeAngle(bearing));
    }

    if( turns.empty() )
        return turns;

    std::sort(turns.begin(), turns.end(),
              [](const TurnPossibility left, const TurnPossibility right) {
                  return left.bearing < right.bearing;
              });
    for( size_t i = 1; i < turns.size(); ++i )
        if( turns[i-1].bearing == turns[i].bearing )
            turns[i].bearing++;

    auto delta = turns.front().bearing;
    for( auto & turn : turns )
        turn.bearing -= delta;

    return turns;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm

#endif // OSRM_GUIDANCE_TURN_CLASSIFICATION_HPP_
