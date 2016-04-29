#include "extractor/guidance/constants.hpp"
#include "extractor/guidance/turn_analysis.hpp"

#include "util/coordinate.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/simple_logger.hpp"

#include <cstddef>
#include <iomanip>
#include <limits>
#include <map>
#include <set>
#include <unordered_set>
#include <unordered_map>

namespace osrm
{
namespace extractor
{
namespace guidance
{

using EdgeData = util::NodeBasedDynamicGraph::EdgeData;

bool requiresAnnouncement(const EdgeData &from, const EdgeData &to)
{
    return !from.IsCompatibleTo(to);
}

TurnAnalysis::TurnAnalysis(const util::NodeBasedDynamicGraph &node_based_graph,
                           const std::vector<QueryNode> &node_info_list,
                           const RestrictionMap &restriction_map,
                           const std::unordered_set<NodeID> &barrier_nodes,
                           const CompressedEdgeContainer &compressed_edge_container,
                           const util::NameTable &name_table)
    : node_based_graph(node_based_graph), node_info_list(node_info_list),
      compressed_edge_container(compressed_edge_container),
      intersection_generator(node_based_graph,
                             restriction_map,
                             barrier_nodes,
                             node_info_list,
                             compressed_edge_container),
      roundabout_handler(node_based_graph, node_info_list, name_table),
      motorway_handler(node_based_graph, node_info_list, name_table),
      turn_handler(node_based_graph, node_info_list, name_table)
{
}

std::vector<TurnOperation> TurnAnalysis::getTurns(const NodeID from_nid, const EdgeID via_eid) const
{
    auto intersection = intersection_generator(from_nid, via_eid);

    // Roundabouts are a main priority. If there is a roundabout instruction present, we process the
    // turn as a roundabout
    if (roundabout_handler.canProcess(from_nid, via_eid, intersection))
    {
        intersection = roundabout_handler(from_nid, via_eid, std::move(intersection));
    }
    else
    {
        // set initial defaults for normal turns and modifier based on angle
        intersection = setTurnTypes(from_nid, via_eid, std::move(intersection));
        if (motorway_handler.canProcess(from_nid, via_eid, intersection))
        {
            intersection = motorway_handler(from_nid, via_eid, std::move(intersection));
        }
        else
        {
            BOOST_ASSERT(turn_handler.canProcess(from_nid, via_eid, intersection));
            intersection = turn_handler(from_nid, via_eid, std::move(intersection));
        }
    }

    static std::unordered_map<BearingClass, int> turn_map;
    static std::unordered_map<EntryClass, int> entry_map;

    auto turn_class =
        guidance::classifyIntersection(node_based_graph.GetTarget(via_eid), intersection,
                                       node_based_graph, compressed_edge_container, node_info_list);

    static std::size_t duplicates = 0;
    static std::size_t duplicates2 = 0;

    if (turn_map.count(turn_class.second) == 0)
    {
        std::cout << "Bearings [" << turn_map.size() << "]:" << turn_class.second.getStringRepresentation() << std::endl;
        turn_map[turn_class.second] = turn_map.size();
    }
    else
    {
        duplicates++;
        if( duplicates % 1000 == 0 )
        {
            std::cout << "Duplicates: " << duplicates << std::endl;
            std::cout << "Turn Classes: " << turn_map.size() << " Entry Classes: " << entry_map.size() << std::endl;
        }
    }

    if (entry_map.count(turn_class.first) == 0)
    {
        std::cout << "Entry: [" << entry_map.size() << "]:" << turn_class.first.getStringRepresentation() << std::endl;
        entry_map[turn_class.first] = entry_map.size();
    }
    else
    {
        duplicates2++;
        if( duplicates2 % 1000 == 0 )
        {
            std::cout << "Duplicates: " << duplicates2 << std::endl;
            std::cout << "Turn Classes: " << turn_map.size() << " Entry Classes: " << entry_map.size() << std::endl;
        }
    }

    std::vector<TurnOperation> turns;
    for (auto road : intersection)
        if (road.entry_allowed)
            turns.emplace_back(road.turn);

    return turns;
}

// Sets basic turn types as fallback for otherwise unhandled turns
Intersection
TurnAnalysis::setTurnTypes(const NodeID from_nid, const EdgeID, Intersection intersection) const
{
    for (auto &road : intersection)
    {
        if (!road.entry_allowed)
            continue;

        const EdgeID onto_edge = road.turn.eid;
        const NodeID to_nid = node_based_graph.GetTarget(onto_edge);

        road.turn.instruction = {TurnType::Turn, (from_nid == to_nid)
                                                     ? DirectionModifier::UTurn
                                                     : getTurnDirection(road.turn.angle)};
    }
    return intersection;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
