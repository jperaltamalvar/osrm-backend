#include "util/coordinate_calculation.hpp"

#ifndef NDEBUG
#include "util/simple_logger.hpp"
#endif
#include "osrm/coordinate.hpp"

#ifndef NDEBUG
#include <bitset>
#endif
#include <iostream>
#include <iomanip>
#include <limits>

namespace osrm
{
namespace util
{


bool Coordinate::IsValid() const
{
    return !(lat > FixedLatitude(90 * COORDINATE_PRECISION) ||
             lat < FixedLatitude(-90 * COORDINATE_PRECISION) ||
             lon > FixedLongitude(180 * COORDINATE_PRECISION) ||
             lon < FixedLongitude(-180 * COORDINATE_PRECISION));
}


bool FloatCoordinate::IsValid() const
{
    return !(lat > FloatLatitude(90) ||
             lat < FloatLatitude(-90) ||
             lon > FloatLongitude(180) ||
             lon < FloatLongitude(-180));
}


bool operator==(const Coordinate lhs, const Coordinate rhs)
{
    return lhs.lat == rhs.lat && lhs.lon == rhs.lon;
}
bool operator==(const FloatCoordinate lhs, const FloatCoordinate rhs)
{
    return lhs.lat == rhs.lat && lhs.lon == rhs.lon;
}

bool operator!=(const Coordinate lhs, const Coordinate rhs) { return !(lhs == rhs); }
bool operator!=(const FloatCoordinate lhs, const FloatCoordinate rhs) { return !(lhs == rhs); }

std::ostream &operator<<(std::ostream &out, const Coordinate coordinate)
{
    out << std::setprecision(12) << "(lon:" << toFloating(coordinate.lon)
        << ", lat:" << toFloating(coordinate.lat) << ")";
    return out;
}
std::ostream &operator<<(std::ostream &out, const FloatCoordinate coordinate)
{
    out << std::setprecision(12) << "(lon:" << coordinate.lon
        << ", lat:" << coordinate.lat << ")";
    return out;
}
}
}
