#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "../../../Helpers/Types.h"

namespace RAPTOR {

class JourneyLegWithStopEvent {

public:
    JourneyLegWithStopEvent(const Vertex from = noVertex, const Vertex to = noVertex, const size_t fromStopEventId = 0, const size_t toStopEventId = 0, const int departureTime = never, const int arrivalTime = never, const bool usesRoute = false, const RouteId routeId = noRouteId)
        : from(from)
        , to(to)
        , fromStopEventId(fromStopEventId)
        , toStopEventId(toStopEventId)
        , departureTime(departureTime)
        , arrivalTime(arrivalTime)
        , usesRoute(usesRoute)
        , routeId(routeId)
    {
    }

    JourneyLegWithStopEvent(const Vertex from, const Vertex to, const size_t fromStopEventId, const size_t toStopEventId, const int departureTime, const int arrivalTime, const Edge edge)
        : from(from)
        , to(to)
        , fromStopEventId(fromStopEventId)
        , toStopEventId(toStopEventId)
        , departureTime(departureTime)
        , arrivalTime(arrivalTime)
        , usesRoute(false)
        , transferId(edge)
    {
    }

    JourneyLegWithStopEvent(const Vertex from = noVertex, const Vertex to = noVertex, const int departureTime = never, const int arrivalTime = never, const bool usesRoute = false, const RouteId routeId = noRouteId)
        : from(from)
        , to(to)
        , fromStopEventId(0)
        , toStopEventId(0)
        , departureTime(departureTime)
        , arrivalTime(arrivalTime)
        , usesRoute(usesRoute)
        , routeId(routeId)
    {
    }

    JourneyLegWithStopEvent(const Vertex from, const Vertex to, const int departureTime, const int arrivalTime, const Edge edge)
        : from(from)
        , to(to)
        , fromStopEventId(0)
        , toStopEventId(0)
        , departureTime(departureTime)
        , arrivalTime(arrivalTime)
        , usesRoute(false)
        , transferId(edge)
    {
    }

    inline int transferTime() const noexcept
    {
        return usesRoute ? 0 : arrivalTime - departureTime;
    }

    inline friend std::ostream& operator<<(std::ostream& out, const JourneyLegWithStopEvent& leg) noexcept
    {
        return out << "from: " << leg.from << ", to: " << leg.to << ", dep-Time: " << leg.departureTime << ", arr-Time: " << leg.arrivalTime << (leg.usesRoute ? ", route: " : ", transfer: ") << leg.routeId << ", fromStopEventId: " << leg.fromStopEventId << ", toStopEventId: " << leg.toStopEventId;
    }

public:
    Vertex from;
    Vertex to;
    size_t fromStopEventId;
    size_t toStopEventId;
    int departureTime;
    int arrivalTime;
    bool usesRoute;
    union {
        RouteId routeId;
        Edge transferId;
    };
};

using JourneyWithStopEvent = std::vector<JourneyLegWithStopEvent>;

inline std::vector<Vertex> journeyToPath(const JourneyWithStopEvent& journey) noexcept
{
    std::vector<Vertex> path;
    for (const JourneyLegWithStopEvent& leg : journey) {
        path.emplace_back(leg.from);
    }
    path.emplace_back(journey.back().to);
    return path;
}

inline int totalTransferTime(const JourneyWithStopEvent& journey) noexcept
{
    int transferTime = 0;
    for (const JourneyLegWithStopEvent& leg : journey) {
        transferTime += leg.transferTime();
    }
    return transferTime;
}

inline int intermediateTransferTime(const JourneyWithStopEvent& journey) noexcept
{
    int transferTime = 0;
    for (size_t i = 1; i < journey.size() - 1; i++) {
        transferTime += journey[i].transferTime();
    }
    return transferTime;
}

inline int initialTransferTime(const JourneyWithStopEvent& journey) noexcept
{
    if (journey.empty())
        return 0;
    int transferTime = journey[0].transferTime();
    if (journey.size() > 1) {
        transferTime += journey.back().transferTime();
    }
    return transferTime;
}

inline size_t countTrips(const JourneyWithStopEvent& journey) noexcept
{
    size_t numTrips = 0;
    for (const JourneyLegWithStopEvent& leg : journey) {
        if (leg.usesRoute && leg.routeId != noRouteId)
            numTrips++;
    }
    return numTrips;
}

}
