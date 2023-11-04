#pragma once

#include <algorithm>
#include <vector>

#include "../../Helpers/Console/Progress.h"
#include "../RAPTOR/Data.h"
#include "../RAPTOR/Entities/RouteSegment.h"
#include "../RAPTOR/Entities/StopEvent.h"
#include "Entities/Lookups.h"
#include "Entities/LowerBound.h"

namespace TransferPattern {

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
        , transferPatternOfStop(data.numberOfStops())
        , lowerBounds(data.numberOfStops())
    {
        // fill all the lowerBounds with enough space
        for (size_t stop(0); stop < data.numberOfStops(); ++stop) {
            lowerBounds[stop].resize(data.numberOfStops());
        }

        buildLineLookup();
        buildStopLookup();
    }

    Data(const std::string& fileName)
    {
        deserialize(fileName);
    }

private:
    inline void buildLineLookup(const bool verbose = true)
    {
        if (verbose)
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

    inline void buildStopLookup(const bool verbose = true)
    {
        if (verbose)
            std::cout << "Building the Stop-Lookup Datastructure" << std::endl;
        Progress progress(raptorData.numberOfRoutes() + raptorData.numberOfStops());

        stopLookup.assign(raptorData.numberOfStops(), {});

        for (const RouteId route : raptorData.routes()) {
            auto stopsOfRoute = raptorData.stopsOfRoute(route);

            for (size_t i(0); i < stopsOfRoute.size(); ++i) {
                stopLookup[stopsOfRoute[i]].incidentLines.push_back(RAPTOR::RouteSegment(route, StopIndex(i)));
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
    inline void assignLowerBounds(const StopId stop, std::vector<int> bestTravelTimes, std::vector<uint8_t> bestMinNumberOfTrips)
    {
        AssertMsg(raptorData.isStop(stop), "Stop is not a valid stop!");
        AssertMsg(bestTravelTimes.size() == raptorData.numberOfStops(), "BestTravelTimes has not the right amount of elements!");
        AssertMsg(bestMinNumberOfTrips.size() == raptorData.numberOfStops(), "BestMinNumberOfTrips has not the right amount of elements!");

        for (size_t i(0); i < raptorData.numberOfStops(); ++i) {
            AssertMsg(stop < lowerBounds[i].travelTimes.size(), "Size is not correct!");
            AssertMsg(stop < lowerBounds[i].numberOfTrips.size(), "Size is not correct!");

            lowerBounds[i].travelTimes[stop] = bestTravelTimes[i];
            lowerBounds[i].numberOfTrips[stop] = bestMinNumberOfTrips[i];
        }
    }

    inline int getLowerBoundTravelTime(const StopId& target, const StopId& u) {
        AssertMsg(raptorData.isStop(target), "Target is not a stop!");
        AssertMsg(raptorData.isStop(u), "Stop u is not a stop!");

        return lowerBounds[target].travelTimes[u];
    }

    inline int getLowerBoundNumberOfTrips(const StopId target, const StopId u) {
        AssertMsg(raptorData.isStop(target), "Target is not a stop!");
        AssertMsg(raptorData.isStop(u), "Stop u is not a stop!");

        return lowerBounds[target].numberOfTrips[u];
    }

    inline RAPTOR::StopEvent getStopEvent(const RouteId routeId = noRouteId, const size_t tripIndex = (size_t)-1, const StopIndex stopIndex = StopIndex(0)) const
    {
        AssertMsg(raptorData.isRoute(routeId), "Route is not a route!");
        AssertMsg(tripIndex != (size_t)-1, "TripIndex is invalid!");
        AssertMsg(StopIndex(0) <= stopIndex && stopIndex < StopIndex(raptorData.numberOfStopsInRoute(routeId)), "StopIndex is out of bounds!");

        return lineLookup[routeId].stopsAlongLine[stopIndex].halts[tripIndex];
    }

    inline int getArrivalTime(const RouteId routeId = noRouteId, const size_t tripIndex = (size_t)-1, const StopIndex stopIndex = StopIndex(0)) const
    {
        return getStopEvent(routeId, tripIndex, stopIndex).arrivalTime;
    }

    inline int getDepartureTime(const RouteId routeId = noRouteId, const size_t tripIndex = (size_t)-1, const StopIndex stopIndex = StopIndex(0)) const
    {
        return getStopEvent(routeId, tripIndex, stopIndex).departureTime;
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

    inline std::pair<size_t, size_t> maxNumVerticesAndNumEdgesInTP() const
    {
        std::pair<size_t, size_t> result(0, 0);
        for (auto& tp : transferPatternOfStop) {
            result.first = std::max(result.first, tp.numVertices());
            result.second = std::max(result.second, tp.numEdges());
        }

        return result;
    }

    inline void printStatsAboutTP() const
    {
        size_t sumVertices(0), sumEdges(0), maxVertices(0), maxEdges(0);
        for (auto& tp : transferPatternOfStop) {
            maxVertices = std::max(tp.numVertices(), maxVertices);
            maxEdges = std::max(tp.numEdges(), maxEdges);

            sumVertices += tp.numVertices();
            sumEdges += tp.numEdges();
        }

        std::cout << "   Total # of vertices:      " << std::setw(12) << String::prettyInt(sumVertices) << std::endl;
        std::cout << "   Total # of edges:         " << std::setw(12) << String::prettyInt(sumEdges) << std::endl;
        std::cout << "   Max # of vertices:        " << std::setw(12) << String::prettyInt(maxVertices) << std::endl;
        std::cout << "   Max # of edges:           " << std::setw(12) << String::prettyInt(maxEdges) << std::endl;
    }

    inline long long byteSize() const noexcept
    {
        long long result = Vector::byteSize(lineLookup);
        result += Vector::byteSize(stopLookup);
        result += raptorData.byteSize();

        for (size_t stop(0); stop < transferPatternOfStop.size(); ++stop)
            result += transferPatternOfStop[stop].byteSize();

        return result;
    }

    inline void printInfo() const noexcept
    {
        std::cout << "Transfer Pattern data:" << std::endl;
        std::cout << "First, the underlying RAPTOR data:" << std::endl;

        raptorData.printInfo();

        std::cout << "Info about Transfer Pattern:" << std::endl;
        printStatsAboutTP();
        std::cout << "   Storage usage of all TP:  " << std::setw(12) << String::bytesToString(byteSize()) << std::endl;
    }

    inline void serialize(const std::string& fileName)
    {
        raptorData.serialize(fileName + ".raptor");
        IO::serialize(fileName, lineLookup, stopLookup, firstTripIdOfLine, lowerBounds);
        IO::serialize(fileName + ".transferPattern", transferPatternOfStop);
    }

    inline void deserialize(const std::string& fileName)
    {
        raptorData.deserialize(fileName + ".raptor");
        IO::deserialize(fileName, lineLookup, stopLookup, firstTripIdOfLine, lowerBounds);

        std::cout << "Loading all transfer patterns from " << fileName << ".transferPattern!" << std::endl;
        IO::deserialize(fileName + ".transferPattern", transferPatternOfStop);
    }

public:
    RAPTOR::Data raptorData;

    std::vector<LookupOfLine> lineLookup;
    std::vector<StopLookup> stopLookup;
    std::vector<TripId> firstTripIdOfLine;

    // StaticDAGTransferPattern holds ViaVertex == points to the correct StopId
    // and holds TravelTime == if negative, the edge is a trip edge
    std::vector<StaticDAGTransferPattern> transferPatternOfStop;

    std::vector<LowerBound> lowerBounds;
};

} // namespace TransferPattern
