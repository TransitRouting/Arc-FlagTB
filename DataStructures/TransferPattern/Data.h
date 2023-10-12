#pragma once

#include <algorithm>
#include <vector>

#include "../../Helpers/Console/Progress.h"
#include "../RAPTOR/Data.h"
#include "../RAPTOR/Entities/StopEvent.h"

namespace TransferPattern {

struct HaltsOfStopInLine {
    HaltsOfStopInLine(std::vector<RAPTOR::StopEvent> halts = {})
        : halts(halts)
    {
    }

    std::vector<RAPTOR::StopEvent> halts;
};

struct LookupOfLine {
    LookupOfLine(std::vector<HaltsOfStopInLine> stopsAlongLine = {})
        : stopsAlongLine(stopsAlongLine)
    {
    }

    std::vector<HaltsOfStopInLine> stopsAlongLine;
};

struct LineAndStopIndex {
    LineAndStopIndex(RouteId routeId = noRouteId, StopIndex stopIndex = noStopIndex)
        : routeId(routeId)
        , stopIndex(stopIndex)
    {
    }

    RouteId routeId;
    StopIndex stopIndex;

    bool operator<(const LineAndStopIndex& a) const
    {
        return std::tie(routeId, stopIndex) < std::tie(a.routeId, a.stopIndex);
    }

    bool beforeOnSameLine(const LineAndStopIndex& a) const
    {
        return routeId == a.routeId && stopIndex < a.stopIndex;
    }
};

struct StopLookup {
    StopLookup(std::vector<LineAndStopIndex> incidentLines = {})
        : incidentLines(incidentLines)
    {
    }

    std::vector<LineAndStopIndex> incidentLines;
};

class Data {
public:
    Data()
    {
    }

    Data(const RAPTOR::Data& data)
        : raptorData(data)
        , lineLookup(data.numberOfRoutes())
        , stopLookup(data.numberOfStops())
        , firstTripIdOfLine(data.numberOfRoutes() + 1, noTripId)
    {
        buildLineLookup();
        buildStopLookup();
    }

    Data(const std::string& fileName)
    {
        deserialize(fileName);
    }

private:
    inline void buildLineLookup()
    {
        std::cout << "Building the Direct-Connection-Lookup Datastructure" << std::endl;
        Progress progress(raptorData.numberOfRoutes());

        lineLookup.assign(raptorData.numberOfRoutes(), {});
        firstTripIdOfLine.assign(raptorData.numberOfRoutes() + 1, noTripId);

        TripId currentFirstTripId(0);
        size_t currentNumberOfTrips = 0;
        size_t currentNumberOfStopsInRoute = 0;

        for (const RouteId route : raptorData.routes()) {
            currentNumberOfTrips = raptorData.numberOfTripsInRoute(route);
            currentNumberOfStopsInRoute = raptorData.numberOfStopsInRoute(route);

            std::vector<RAPTOR::StopEvent> reserveVector(currentNumberOfTrips);
            lineLookup[route].stopsAlongLine.assign(currentNumberOfStopsInRoute, reserveVector);

            firstTripIdOfLine[route] = currentFirstTripId;
            currentFirstTripId += currentNumberOfTrips;

            for (size_t tripIndex(0); tripIndex < currentNumberOfTrips; ++tripIndex) {
                size_t stopsOfRouteIndex(0);
                for (auto stopEvent : raptorData.stopEventsOfTrip(route, tripIndex)) {
                    AssertMsg(tripIndex < lineLookup[route].stopsAlongLine[stopsOfRouteIndex].halts.size(), "TripIndex is too big for the halts vector!");
                    lineLookup[route].stopsAlongLine[stopsOfRouteIndex].halts[tripIndex] = stopEvent;
                    ++stopsOfRouteIndex;
                }
            }
            ++progress;
        }

        firstTripIdOfLine[raptorData.numberOfRoutes()] = currentFirstTripId;

        progress.finished();
    }

    inline void buildStopLookup()
    {
        std::cout << "Building the Stop-Lookup Datastructure" << std::endl;
        Progress progress(raptorData.numberOfRoutes() + raptorData.numberOfStops());

        stopLookup.assign(raptorData.numberOfStops(), {});

        for (const RouteId route : raptorData.routes()) {
            auto stopsOfRoute = raptorData.stopsOfRoute(route);

            for (size_t i(0); i < stopsOfRoute.size(); ++i) {
                stopLookup[stopsOfRoute[i]].incidentLines.push_back(LineAndStopIndex(route, StopIndex(i)));
            }

            ++progress;
        }

        for (const StopId stop : raptorData.stops()) {
            std::sort(stopLookup[stop].incidentLines.begin(), stopLookup[stop].incidentLines.end());
            ++progress;
        }

        progress.finished();
    }

public:
    inline RAPTOR::StopEvent getStopEvent(const RouteId routeId = noRouteId, const size_t tripIndex = (size_t) -1, const StopIndex stopIndex = noStopIndex) const {
        AssertMsg(raptorData.isRoute(routeId), "Route is not a route!");
        AssertMsg(tripIndex != (size_t) -1, "TripIndex is invalid!");
        AssertMsg(stopIndex < raptorData.numberOfStopsInRoute(routeId), "StopIndex is out of bounds!");

        return lineLookup[routeId].stopsAlongLine[stopIndex].halts[tripIndex];
    }

    inline int getArrivalTime(const RouteId routeId = noRouteId, const size_t tripIndex = (size_t) -1, const StopIndex stopIndex = noStopIndex) const {
        return getStopEvent(routeId, tripIndex, stopIndex).arrivalTime;
    }

    inline int getDepartureTime(const RouteId routeId = noRouteId, const size_t tripIndex = (size_t) -1, const StopIndex stopIndex = noStopIndex) const {
        return getStopEvent(routeId, tripIndex, stopIndex).departureTime;
    }

    inline std::vector<LineAndStopIndex> getStopLookup(const StopId stop = noStop)
    {
        AssertMsg(stop < stopLookup.size(), "Stop is out-of-bounds!");
        AssertMsg(raptorData.isStop(stop), "Stop is not a stop!");

        return stopLookup[stop].incidentLines;
    }

    inline size_t earliestTripIndexOfLineByStopIndex(const StopIndex stopIndex, const RouteId route, const int departureTime = 0) const
    {
        AssertMsg(raptorData.isRoute(route), "Route is not a route!");

        for (size_t tripIndex(0); tripIndex < raptorData.numberOfTripsInRoute(route); ++tripIndex) {
            if (getDepartureTime(route, tripIndex, stopIndex) >= departureTime)
                return tripIndex;
        }

        return (size_t)-1;
    }

    inline TripId tripIdOfLineByTripIndex(const RouteId route = noRouteId, const size_t tripIndex = 0)
    {
        AssertMsg(tripIndex != (size_t)-1, "TripIndex is -1?");
        AssertMsg(raptorData.isRoute(route), "Route is not a route!");

        return TripId(firstTripIdOfLine[route] + tripIndex);
    }

    inline std::pair<TripId, int> directConnectionIntersection(const StopId source = noStop, const StopId target = noStop, const int departureTime = 0)
    {
        AssertMsg(raptorData.isStop(source), "Source is not a valid stop!");
        AssertMsg(raptorData.isStop(target), "Target is not a valid stop!");

        auto sourceStopLookup = getStopLookup(source);
        auto targetStopLookup = getStopLookup(target);

        std::pair<TripId, int> result = std::make_pair(noTripId, INFTY);

        size_t i(0);
        size_t j(0);

        size_t firstReachableTripIndex(-1);
        int currentArrivalTime(-1);

        while (i < sourceStopLookup.size() && j < targetStopLookup.size()) {
            if (sourceStopLookup[i].beforeOnSameLine(targetStopLookup[j])) [[unlikely]] {
                // check if sourceStopLookup[i] is reachable
                firstReachableTripIndex = earliestTripIndexOfLineByStopIndex(sourceStopLookup[i].stopIndex, sourceStopLookup[i].routeId, departureTime);

                if (firstReachableTripIndex == (size_t) -1)
                    continue;
                currentArrivalTime = getArrivalTime(sourceStopLookup[i].routeId, firstReachableTripIndex, targetStopLookup[j].stopIndex);
                if (currentArrivalTime < result.second) 
                    result = std::make_pair(tripIdOfLineByTripIndex(sourceStopLookup[i].routeId), currentArrivalTime);

                ++i;
                ++j;
            } else {
                if (sourceStopLookup[i] < targetStopLookup[j])
                    ++i;
                else
                    ++j;
            }
        }

        return result;
    }

public:
    inline void serialize(const std::string& fileName) const noexcept
    {
        raptorData.serialize(fileName + ".raptor");
        IO::serialize(fileName, lineLookup, stopLookup, firstTripIdOfLine);
    }

    inline void deserialize(const std::string& fileName) noexcept
    {
        raptorData.deserialize(fileName + ".raptor");
        IO::deserialize(fileName, lineLookup, stopLookup, firstTripIdOfLine);
    }

public:
    RAPTOR::Data raptorData;

    std::vector<LookupOfLine> lineLookup;
    std::vector<StopLookup> stopLookup;
    std::vector<TripId> firstTripIdOfLine;
};

} // namespace TransferPattern
