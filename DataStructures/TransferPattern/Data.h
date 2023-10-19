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

    inline void serialize(IO::Serialization& serialize) const
    {
        serialize(halts);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(halts);
    }

    std::vector<RAPTOR::StopEvent> halts;
};

struct LookupOfLine {
    LookupOfLine(std::vector<HaltsOfStopInLine> stopsAlongLine = {})
        : stopsAlongLine(stopsAlongLine)
    {
    }

    inline void serialize(IO::Serialization& serialize) const
    {
        serialize(stopsAlongLine);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(stopsAlongLine);
    }

    std::vector<HaltsOfStopInLine> stopsAlongLine;
};

struct LineAndStopIndex {
    LineAndStopIndex(RouteId routeId = noRouteId, StopIndex stopIndex = noStopIndex)
        : routeId(routeId)
        , stopIndex(stopIndex)
    {
    }

    inline void serialize(IO::Serialization& serialize)
    {
        serialize(routeId, stopIndex);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(routeId, stopIndex);
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

    inline void serialize(IO::Serialization& serialize) const
    {
        serialize(incidentLines);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(incidentLines);
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
        , transferPatternOfStop(data.numberOfStops())
    {
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

public:
    inline size_t maxNumberOfVerticesInTP() const
    {
        size_t maxNumber(0);

        for (auto& tp : transferPatternOfStop) {
            maxNumber = std::max(maxNumber, tp.numVertices());
        }

        return maxNumber;
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
        std::cout << "   Max # vertices in a TP:   " << std::setw(12) << String::prettyInt(maxNumberOfVerticesInTP()) << std::endl;
        std::cout << "   Storage usage of all TP:  " << std::setw(12) << String::bytesToString(byteSize()) << std::endl;
    }

    inline void serialize(const std::string& fileName)
    {
        raptorData.serialize(fileName + ".raptor");
        IO::serialize(fileName, lineLookup, stopLookup, firstTripIdOfLine);
        IO::serialize(fileName + ".transferPattern", transferPatternOfStop);
    }

    inline void deserialize(const std::string& fileName)
    {
        raptorData.deserialize(fileName + ".raptor");
        IO::deserialize(fileName, lineLookup, stopLookup, firstTripIdOfLine);
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
};

} // namespace TransferPattern
