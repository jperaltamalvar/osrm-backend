#include "extractor/guidance/turn_classification.hpp"

namespace osrm
{
namespace extractor
{
namespace guidance
{

std::pair<EntryClass, BearingClass>
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
        turns.emplace_back(road.entry_allowed,
                           (bearing + (0.5 * discrete_angle_step_size)) / discrete_angle_step_size);
    }

    if (turns.empty())
        return turns;

    std::sort(turns.begin(), turns.end(),
              [](const TurnPossibility left, const TurnPossibility right) {
                  return left.bearing < right.bearing;
              });

    /*
    auto delta = turns.front().bearing;
    for (auto &turn : turns)
        turn.bearing -= delta;
    */

    EntryClass entry_class;
    for( std::size_t i = 0; i < turns.size(); ++i )
        entry_class.activate(i);

    BearingClass bearing_class;
    for( std::size_t i = 0; i < turns.size(); ++i )
        bearing_class.addContinuous(turns[i].bearing);

    return turns;
}

} // namespace guidance
} // namespace extractor
} // namespace osrm
