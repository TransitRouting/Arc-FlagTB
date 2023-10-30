/**********************************************************************************

 Copyright (c) 2023 Patrick Steil

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/
#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "../../../../Helpers/Helpers.h"
#include "../../../../Helpers/Meta.h"
#include "../../../../Helpers/Vector/Permutation.h"
#include "../../../../Helpers/Vector/Vector.h"

#include "../../../../DataStructures/Container/Set.h"
#include "../../../../DataStructures/Geometry/Rectangle.h"
#include "../../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"

#include "../../../../DataStructures/Graph/Graph.h"
#include "../../../RAPTOR/Profiler.h"
#include "TransitiveRAPTORModule.h"

namespace RAPTOR::RangeRAPTOR {

template <typename RAPTOR_MODULE, bool ONE_TO_ONE = false, bool DEBUG = false, bool CORRECT_DEPARTURE_TIMES = true>
class RangeRAPTOR {

public:
    using RaptorModule = RAPTOR_MODULE;
    static constexpr bool OneToOne = ONE_TO_ONE;
    static constexpr bool Debug = DEBUG;
    static constexpr bool CorrectDepartureTimes = CORRECT_DEPARTURE_TIMES;
    using InitialTransferGraph = typename RaptorModule::InitialTransferGraph;
    using Type = RangeRAPTOR<RaptorModule, OneToOne, Debug, CorrectDepartureTimes>;

private:
    struct DepartureLabel {
        DepartureLabel(const int departureTime, const StopId departureStop)
            : departureTime(departureTime)
            , departureStop(departureStop)
        {
        }
        inline bool operator<(const DepartureLabel& other) const noexcept
        {
            return departureTime > other.departureTime;
        }
        int departureTime;
        StopId departureStop;
    };

public:
    template <typename ATTRIBUTE>
    RangeRAPTOR(Data& data, const InitialTransferGraph& forwardGraph, const ATTRIBUTE weight)
        : forwardRaptor(data, forwardGraph, weight)
        , data(data)
        , sourceVertex(noVertex)
        , minDepartureTime(never)
        , maxDepartureTime(never)
    {
    }

    template <typename T = TransferGraph, typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    RangeRAPTOR(Data& data, const TransferGraph& forwardGraph)
        : RangeRAPTOR(data, forwardGraph, TravelTime)
    {
    }

    template <typename T = TransferGraph, typename = std::enable_if_t<Meta::Equals<T, TransferGraph>() && Meta::Equals<T, InitialTransferGraph>()>>
    RangeRAPTOR(Data& data)
        : RangeRAPTOR(data, data.transferGraph)
    {
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && !T>>
    inline void runOneToAll(const Vertex source, const int minTime = 0, const int maxTime = 60 * 60 * 24, const size_t maxRounds = INFTY) noexcept
    {
        run(source, IndexedSet<false, Vertex>(Construct::Complete, data.transferGraph.numVertices()), minTime, maxTime, maxRounds);
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && !T>>
    inline void runOneToAllStops(const Vertex source, const int minTime = 0, const int maxTime = 60 * 60 * 24, const size_t maxRounds = INFTY) noexcept
    {
        run(source, IndexedSet<false, Vertex>(Construct::Complete, data.numberOfStops()), minTime, maxTime, maxRounds);
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && !T>>
    inline void run(const Vertex source, const std::vector<Vertex>& targets, const int minTime = 0, const int maxTime = 60 * 60 * 24, const size_t maxRounds = INFTY) noexcept
    {
        run(source, IndexedSet<false, Vertex>(data.transferGraph.numVertices(), targets), minTime, maxTime, maxRounds);
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && !T>>
    inline void run(const Vertex source, const IndexedSet<false, Vertex>& targets, const int minTime = 0, const int maxTime = 60 * 60 * 24, const size_t maxRounds = INFTY) noexcept
    {
        clear();
        sourceVertex = source;
        targetVertices = targets;
        minDepartureTime = minTime;
        maxDepartureTime = maxTime;

        allJourneys.clear();

        if constexpr (CorrectDepartureTimes) {
            forwardRaptor.template run<true>(source, maxDepartureTime, noVertex, maxRounds);
            collectArrivals<false>(targets, forwardRaptor.getReachedVertices());
        } else {
            forwardRaptor.template runInitialize<true>(source, maxDepartureTime);
            forwardRaptor.template runInitialTransfers();
        }

        collectDepartures();
        for (size_t i = 0; i < departures.size(); ++i) {
            forwardRaptor.template runInitialize<false>(source, departures[i].departureTime);
            forwardRaptor.runAddSource(departures[i].departureStop, departures[i].departureTime + forwardRaptor.getWalkingTravelTime(departures[i].departureStop));
            while (i + 1 < departures.size() && departures[i].departureTime == departures[i + 1].departureTime) {
                ++i;
                forwardRaptor.runAddSource(departures[i].departureStop, departures[i].departureTime + forwardRaptor.getWalkingTravelTime(departures[i].departureStop));
            }
            if constexpr (Debug)
                std::cout << "Departure Time: " << departures[i].departureTime << std::endl;
            forwardRaptor.template runRounds(maxRounds);
            collectArrivals(targets, forwardRaptor.getReachedVertices());
        }
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && T>>
    inline void run(const Vertex source, const Vertex target, const int minTime = 0, const int maxTime = 60 * 60 * 24, const size_t maxRounds = INFTY) noexcept
    {
        clear();
        sourceVertex = source;
        minDepartureTime = minTime;
        maxDepartureTime = maxTime;

        if constexpr (CorrectDepartureTimes) {
            forwardRaptor.template run<true>(source, maxDepartureTime, target, maxRounds);
            collectArrivals<false>(target);
        } else {
            forwardRaptor.template runInitialize<true>(source, 0, target);
            forwardRaptor.template runInitialTransfers();
        }

        collectDepartures();
        for (size_t i = 0; i < departures.size(); ++i) {
            forwardRaptor.template runInitialize<false>(source, departures[i].departureTime, target);
            forwardRaptor.runAddSource(departures[i].departureStop, departures[i].departureTime + forwardRaptor.getWalkingTravelTime(departures[i].departureStop));
            while (i + 1 < departures.size() && departures[i].departureTime == departures[i + 1].departureTime) {
                ++i;
                forwardRaptor.runAddSource(departures[i].departureStop, departures[i].departureTime + forwardRaptor.getWalkingTravelTime(departures[i].departureStop));
            }
            if constexpr (Debug)
                std::cout << "Departure Time: " << departures[i].departureTime << std::endl;
            forwardRaptor.template runRounds(maxRounds);
            if (forwardRaptor.targetReached()) {
                collectArrivals(target);
            }
        }
    }

    inline void reset() noexcept
    {
        forwardRaptor.reset();
    }

private:
    inline void clear() noexcept
    {
    }

    inline void collectDepartures() noexcept
    {
        departures.clear();
        for (const RouteId route : data.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* stopEvents = data.firstTripOfRoute(route);
            for (uint32_t stopEventIndex = 0; stopEventIndex < data.numberOfStopEventsInRoute(route); ++stopEventIndex) {
                if ((stopEventIndex + 1) % numberOfStops == 0)
                    continue;
                const StopId stop = stops[stopEventIndex % numberOfStops];
                const int walkingTime = forwardRaptor.getWalkingTravelTime(stop);
                if (walkingTime == intMax)
                    continue;
                const int departureTime = stopEvents[stopEventIndex].departureTime - walkingTime;
                if (departureTime < minDepartureTime)
                    continue;
                if constexpr (CorrectDepartureTimes) {
                    if (departureTime >= maxDepartureTime)
                        continue;
                }
                departures.emplace_back(departureTime, stop);
            }
        }
        sort(departures);
        if constexpr (Debug)
            std::cout << "Number of departures: " << departures.size() << std::endl;
    }

    template <bool RUN_BACKWARD_QUERIES = false>
    inline void collectArrivals(const IndexedSet<false, Vertex>& targets, const IndexedSet<false, Vertex>& reachedVertices) noexcept
    {
        const IndexedSet<false, Vertex>& smallTargetSet = (targets.size() < reachedVertices.size()) ? targets : reachedVertices;
        const IndexedSet<false, Vertex>& largeTargetSet = (targets.size() < reachedVertices.size()) ? reachedVertices : targets;
        for (const Vertex target : smallTargetSet) {
            if (!largeTargetSet.contains(target))
                continue;
            collectArrivals<RUN_BACKWARD_QUERIES>(target);
        }
    }

    template <bool RUN_BACKWARD_QUERIES = false>
    inline void collectArrivals(const Vertex target) noexcept
    {
        std::vector<JourneyWithStopEvent> journeys = forwardRaptor.getJourneys(target);

        allJourneys.insert(allJourneys.end(), journeys.begin(), journeys.end());
    }

public:
    inline std::vector<JourneyWithStopEvent> getAllJourneys() const noexcept
    {
        return allJourneys;
    }

private:
    RaptorModule forwardRaptor;
    Data& data;

    Vertex sourceVertex;
    int minDepartureTime;
    int maxDepartureTime;
    std::vector<DepartureLabel> departures;

    // One-to- ;
    IndexedSet<false, Vertex> targetVertices;

    std::vector<JourneyWithStopEvent> allJourneys;
};

}
