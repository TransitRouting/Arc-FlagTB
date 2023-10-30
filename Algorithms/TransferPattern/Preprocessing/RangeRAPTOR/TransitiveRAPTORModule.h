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

#include "../../../../Helpers/Vector/Vector.h"

#include "../../../../DataStructures/Container/ExternalKHeap.h"
#include "../../../../DataStructures/Container/Map.h"
#include "../../../../DataStructures/Container/Set.h"
#include "../../../../DataStructures/Intermediate/Data.h"
#include "../../../../DataStructures/RAPTOR/Data.h"
#include "../../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../../DataStructures/RAPTOR/Entities/JourneyWithStopEvent.h"

namespace RAPTOR::RangeRAPTOR {

using EdgeListOfStopEventIds = std::vector<std::pair<size_t, size_t>>;

template <typename PROFILER, bool ONE_TO_ONE = false, bool COLLECT_REACHED_VERTICES = true, bool TRIP_PRUNING = true>
class TransitiveRAPTORModule {

public:
    using Profiler = PROFILER;
    static constexpr bool OneToOne = ONE_TO_ONE;
    static constexpr bool CollectReachedVertices = COLLECT_REACHED_VERTICES;
    static constexpr bool TripPruning = TRIP_PRUNING;
    using Type = TransitiveRAPTORModule<Profiler, OneToOne, CollectReachedVertices, TripPruning>;
    using InitialTransferGraph = TransferGraph;

public:
    struct EarliestArrivalLabel {
        EarliestArrivalLabel()
            : arrivalTime(never)
            , parentDepartureTime(never)
            , parent(noVertex)
            , usesRoute(false)
            , routeId(noRouteId)
            , timestamp(0)
            , fromStopEventId(0)
            , toStopEventId(0)
        {
        }
        int arrivalTime;
        int parentDepartureTime;
        Vertex parent;
        bool usesRoute;
        RouteId routeId;
        int timestamp;
        size_t fromStopEventId;
        size_t toStopEventId;
    };
    using Round = std::vector<EarliestArrivalLabel>;

public:
    TransitiveRAPTORModule(Data& data, const Profiler& profilerTemplate = Profiler())
        : data(data)
        , roundIndex(-1)
        , stopsUpdatedByRoute(data.numberOfStops())
        , stopsUpdatedByTransfer(data.numberOfStops())
        , stopsUpdatedWithTarget(data.numberOfStops(), false)
        , routesServingUpdatedStops(data.numberOfRoutes())
        , sourceStop(noStop)
        , targetStop(noStop)
        , sourceDepartureTime(intMax)
        , timestamp(0)
        , initialTransferDistance(data.numberOfStops(), INFTY)
        , profiler(profilerTemplate)
        , reachedVertices(data.numberOfStops())
    {
        AssertMsg(data.hasImplicitBufferTimes(), "Departure buffer times have to be implicit!");
        if constexpr (TripPruning)
            resetLastTrip();
        profiler.registerExtraRounds({ EXTRA_ROUND_CLEAR, EXTRA_ROUND_INITIALIZATION });
        profiler.registerPhases({ PHASE_INITIALIZATION, PHASE_COLLECT, PHASE_SCAN, PHASE_TRANSFERS });
        profiler.registerMetrics({ METRIC_ROUTES, METRIC_ROUTE_SEGMENTS, METRIC_EDGES, METRIC_STOPS_BY_TRIP, METRIC_STOPS_BY_TRANSFER });
        profiler.initialize();
    }

    template <typename ATTRIBUTE>
    TransitiveRAPTORModule(Data& data, const InitialTransferGraph&, const ATTRIBUTE)
        : TransitiveRAPTORModule(data)
    {
    }

    template <bool CLEAR = true>
    inline void run(const Vertex source, const int departureTime, const Vertex target = noVertex, const size_t maxRounds = INFTY) noexcept
    {
        runInitialize<CLEAR>(source, departureTime, target);
        runInitialTransfers();
        evaluateInitialTransfers();
        runRounds(maxRounds);
    }

    template <bool CLEAR = true>
    inline void runInitialize(const Vertex source, const int departureTime, const Vertex target = noVertex) noexcept
    {
        profiler.start();
        profiler.startExtraRound(EXTRA_ROUND_CLEAR);
        clear<CLEAR>();
        profiler.doneRound();

        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        initialize(source, departureTime, target);
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.doneRound();
    }

    inline void runInitialTransfers() noexcept
    {
        profiler.startExtraRound(EXTRA_ROUND_INITIALIZATION);
        profiler.startPhase();
        relaxInitialTransfers();
        profiler.donePhase(PHASE_TRANSFERS);
        profiler.doneRound();
    }

    inline void runAddSource(const StopId source, const int departureTime) noexcept
    {
        AssertMsg(departureTime >= 0, "Negative depTime??");
        EarliestArrivalLabel& label = currentRoundLabel(source);
        label.arrivalTime = departureTime;
        label.parentDepartureTime = sourceDepartureTime;
        label.usesRoute = false;
        stopsUpdatedByTransfer.insert(source);
    }

    inline void runRounds(const size_t maxRounds = INFTY) noexcept
    {
        profiler.startRound();
        profiler.startPhase();
        startNewRound();
        profiler.donePhase(PHASE_INITIALIZATION);
        profiler.startPhase();
        collectRoutesServingUpdatedStops();
        profiler.donePhase(PHASE_COLLECT);
        profiler.startPhase();
        scanRoutes();
        profiler.donePhase(PHASE_SCAN);
        for (size_t i = 0; (i < maxRounds) && (!stopsUpdatedByRoute.empty()); ++i) {
            profiler.startPhase();
            relaxIntermediateTransfers();
            profiler.donePhase(PHASE_TRANSFERS);
            profiler.doneRound();
            profiler.startRound();
            profiler.startPhase();
            startNewRound();
            profiler.donePhase(PHASE_INITIALIZATION);
            profiler.startPhase();
            collectRoutesServingUpdatedStops();
            profiler.donePhase(PHASE_COLLECT);
            profiler.startPhase();
            scanRoutes();
            profiler.donePhase(PHASE_SCAN);
        }
        profiler.doneRound();
        profiler.done();
    }

    inline std::vector<JourneyWithStopEvent> getJourneys(const Vertex vertex) noexcept
    {
        std::vector<JourneyWithStopEvent> journeys;
        for (size_t i = 0; i <= roundIndex; ++i) {
            getJourney(journeys, i, vertex);
        }
        return journeys;
    }

    inline std::vector<EdgeListOfStopEventIds> getEdgeLists(const Vertex vertex) noexcept
    {
        std::vector<JourneyWithStopEvent> journeys;
        std::vector<EdgeListOfStopEventIds> edgeLists;
        for (size_t i = 0; i <= roundIndex; ++i) {
            getEdgeList(journeys, edgeLists, i, vertex);
        }
        return edgeLists;
    }

    inline bool getJourney(const Vertex vertex, const size_t round, JourneyWithStopEvent& journey) noexcept
    {
        std::vector<JourneyWithStopEvent> journeys;
        getJourney(journeys, round, vertex);
        if (journeys.empty())
            return false;
        journey = journeys[0];
        return true;
    }

    inline JourneyWithStopEvent getEarliestJourney(const Vertex vertex) noexcept
    {
        std::vector<JourneyWithStopEvent> journeys = getJourneys(vertex);
        return journeys.empty() ? JourneyWithStopEvent() : journeys.back();
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && T>>
    inline std::vector<ArrivalLabel> getArrivals() noexcept
    {
        return getArrivals(targetStop);
    }

    inline std::vector<ArrivalLabel> getArrivals(const Vertex vertex) noexcept
    {
        std::vector<ArrivalLabel> labels;
        for (size_t i = 0; i <= roundIndex; ++i) {
            getArrival(labels, i, vertex);
        }
        return labels;
    }

    inline bool reachable(const Vertex vertex) noexcept
    {
        AssertMsg(data.transferGraph.isVertex(vertex), "The Id " << vertex << " does not correspond to any vertex!");
        return getEarliestArrivalTime(vertex) < never;
    }

    inline int getEarliestArrivalTime(const Vertex vertex) noexcept
    {
        AssertMsg(data.isStop(vertex), "The Id " << vertex << " does not correspond to any stop!");
        return roundLabel(rounds.size() - 1, StopId(vertex)).arrivalTime;
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && T>>
    inline int getWalkingArrivalTime() const noexcept
    {
        return sourceDepartureTime + initialTransferDistance[targetStop];
    }

    inline int getWalkingArrivalTime(const Vertex vertex) noexcept
    {
        return sourceDepartureTime + initialTransferDistance[vertex];
    }

    template <bool T = OneToOne, typename = std::enable_if_t<T == OneToOne && T>>
    inline int getWalkingTravelTime() const noexcept
    {
        return initialTransferDistance[targetStop];
    }

    inline int getWalkingTravelTime(const Vertex vertex) noexcept
    {
        return initialTransferDistance[vertex];
    }

    inline std::vector<Vertex> getPath(const Vertex vertex)
    {
        AssertMsg(data.transferGraph.isVertex(vertex), "The Id " << vertex << " does not correspond to any vertex!");
        return journeyToPath(getJourneys(vertex).back());
    }

    inline std::vector<std::string> getRouteDescription(const Vertex vertex)
    {
        AssertMsg(data.transferGraph.isVertex(vertex), "The Id " << vertex << " does not correspond to any vertex!");
        return data.journeyToText(getJourneys(vertex).back());
    }

    inline int getArrivalTime(const Vertex vertex, const size_t numberOfTrips) noexcept
    {
        AssertMsg(data.isStop(StopId(vertex)), "Stop is not really a stop!\n");
        AssertMsg(roundLabel(numberOfTrips, StopId(vertex)).arrivalTime < never, "No label found for stop " << vertex << " in round " << numberOfTrips << "!");
        return roundLabel(numberOfTrips, StopId(vertex)).arrivalTime;
    }

    inline const IndexedSet<false, Vertex>& getReachedVertices() const noexcept
    {
        return reachedVertices;
    }

    inline bool targetReached() const noexcept
    {
        return reachedVertices.contains(targetStop);
    }

    template <bool RESET = true>
    inline void clear() noexcept
    {
        roundIndex = -1;
        stopsUpdatedByRoute.clear();
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        sourceStop = noStop;
        targetStop = noStop;
        reachedVertices.clear();
        if constexpr (RESET) {
            if constexpr (TripPruning)
                resetLastTrip();
            timestamp = 1;
            std::vector<Round>().swap(rounds);
            Vector::fill(initialTransferDistance, INFTY);
        } else {
            ++timestamp;
        }
    }

    inline void reset() noexcept
    {
        clear<true>();
    }

    inline const Profiler& getProfiler() const noexcept
    {
        return profiler;
    }

    template <bool T = TripPruning, typename = std::enable_if_t<T == TripPruning && T>>
    inline void resetLastTrip() noexcept
    {
        std::vector<std::vector<std::vector<uint16_t>>>(1).swap(lastTrip);
        std::vector<std::vector<uint16_t>>(data.numberOfRoutes()).swap(lastTrip[0]);
        for (const RouteId route : data.routes()) {
            std::vector<uint16_t>(data.numberOfStopsInRoute(route) - 1, data.numberOfTripsInRoute(route) - 1).swap(lastTrip[0][route]);
        }
    }

    inline void initialize(const Vertex source, const int departureTime, const Vertex target) noexcept
    {
        AssertMsg(departureTime >= 0, "Nothing departs before the big bang!");
        sourceStop = StopId(source);
        targetStop = StopId(target);
        sourceDepartureTime = departureTime;
        startNewRound();
        stopsUpdatedByRoute.insert(sourceStop);
        if constexpr (CollectReachedVertices)
            reachedVertices.insert(source);
        EarliestArrivalLabel& label = currentRoundLabel(sourceStop);
        label.arrivalTime = departureTime;
        label.parent = source;
        label.parentDepartureTime = departureTime;
        label.usesRoute = false;
    }

    inline void evaluateInitialTransfers() noexcept
    {
        routesServingUpdatedStops.clear();
        stopsUpdatedByTransfer.clear();
        stopsUpdatedByTransfer.insert(sourceStop);
        if constexpr (CollectReachedVertices)
            reachedVertices.insert(sourceStop);
        for (const Edge edge : data.transferGraph.edgesFrom(sourceStop)) {
            const Vertex stop = data.transferGraph.get(ToVertex, edge);
            AssertMsg(data.isStop(stop), "Stop is not really a stop!\n");
            roundLabel(0, StopId(stop));
            stopsUpdatedByTransfer.insert(StopId(stop));
            if constexpr (CollectReachedVertices)
                reachedVertices.insert(stop);
        }
    }

    inline void collectRoutesServingUpdatedStops() noexcept
    {
        for (const StopId stop : stopsUpdatedByTransfer) {
            for (const RouteSegment& route : data.routesContainingStop(stop)) {
                AssertMsg(data.isRoute(route.routeId), "Route " << route.routeId << " is out of range!");
                AssertMsg(data.stopIds[data.firstStopIdOfRoute[route.routeId] + route.stopIndex] == stop, "RAPTOR data contains invalid route segments!");
                if (route.stopIndex + 1 == data.numberOfStopsInRoute(route.routeId))
                    continue;
                if constexpr (TripPruning) {
                    const uint16_t tripNum = lastTrip[roundIndex][route.routeId][route.stopIndex];
                    if (tripNum == uint16_t(-1))
                        continue;
                    const StopEvent* trip = data.tripOfRoute(route.routeId, tripNum) + route.stopIndex;
                    if (trip->departureTime < previousRoundLabel(stop).arrivalTime)
                        continue;
                } else {
                    if (data.lastTripOfRoute(route.routeId)[route.stopIndex].departureTime < previousRoundLabel(stop).arrivalTime)
                        continue;
                }
                if (routesServingUpdatedStops.contains(route.routeId)) {
                    routesServingUpdatedStops[route.routeId] = std::min(routesServingUpdatedStops[route.routeId], route.stopIndex);
                } else {
                    routesServingUpdatedStops.insert(route.routeId, route.stopIndex);
                }
            }
        }
    }

    inline size_t getStopEventId(const StopEvent* stopEvent)
    {
        return size_t(stopEvent - &(data.stopEvents[0]));
    }

    inline void scanRoutes() noexcept
    {
        stopsUpdatedByRoute.clear();
        for (const RouteId route : routesServingUpdatedStops.getKeys()) {
            profiler.countMetric(METRIC_ROUTES);
            StopIndex stopIndex = routesServingUpdatedStops[route];
            const size_t tripSize = data.numberOfStopsInRoute(route);
            AssertMsg(stopIndex < tripSize - 1, "Cannot scan a route starting at/after the last stop (Route: " << route << ", StopIndex: " << stopIndex << ", TripSize: " << tripSize << ", RoundIndex: " << roundIndex << ")!");

            const StopId* stops = data.stopArrayOfRoute(route);
            const StopEvent* firstTrip = data.firstTripOfRoute(route);
            const StopEvent* trip;
            if constexpr (TripPruning) {
                trip = data.tripOfRoute(route, lastTrip[roundIndex][route][stopIndex]);
                if (trip < firstTrip)
                    continue;
            } else {
                trip = data.lastTripOfRoute(route);
            }
            StopId stop = stops[stopIndex];
            AssertMsg(trip[stopIndex].departureTime >= previousRoundLabel(stop).arrivalTime, "Cannot scan a route after the last trip has departed (Route: " << route << ", Stop: " << stop << ", StopIndex: " << stopIndex << ", Time: " << previousRoundLabel(stop).arrivalTime << ", LastDeparture: " << trip[stopIndex].departureTime << ", RoundIndex: " << roundIndex << ")!");

            StopIndex parentIndex = stopIndex;
            while (stopIndex < tripSize - 1) {
                while ((trip > firstTrip) && ((trip - tripSize + stopIndex)->departureTime >= previousRoundLabel(stop).arrivalTime)) {
                    trip -= tripSize;
                    parentIndex = stopIndex;
                }
                if constexpr (TripPruning)
                    lastTrip[roundIndex][route][stopIndex] = uint16_t(((trip - firstTrip) / tripSize));
                ++stopIndex;
                stop = stops[stopIndex];
                profiler.countMetric(METRIC_ROUTE_SEGMENTS);
                if (arrivalByRoute(stop, trip[stopIndex].arrivalTime)) {
                    if constexpr (CollectReachedVertices)
                        reachedVertices.insert(stop);
                    EarliestArrivalLabel& label = currentRoundLabel(stop);
                    AssertMsg(data.isStop(stops[parentIndex]), "ParentStop is not a stop!\n");
                    label.parent = stops[parentIndex];
                    label.parentDepartureTime = trip[parentIndex].departureTime;
                    label.usesRoute = true;
                    label.routeId = route;
                    // to get the vertex id afterwards
                    label.fromStopEventId = getStopEventId(&trip[parentIndex]);
                    label.toStopEventId = getStopEventId(&trip[stopIndex]);
                }
            }
        }

        stopsUpdatedByRoute.sort();
    }

    inline void relaxInitialTransfers() noexcept
    {
        for (const Edge edge : data.transferGraph.edgesFrom(sourceStop)) {
            profiler.countMetric(METRIC_EDGES);
            const Vertex stop = data.transferGraph.get(ToVertex, edge);
            AssertMsg(data.isStop(stop), "Graph contains edges to non stop vertices!");
            initialTransferDistance[stop] = data.transferGraph.get(TravelTime, edge);
        }
        initialTransferDistance[sourceStop] = 0;
    }

    inline void relaxIntermediateTransfers() noexcept
    {
        stopsUpdatedByTransfer.clear();
        routesServingUpdatedStops.clear();
        for (const StopId stop : stopsUpdatedByRoute) {
            const int earliestArrivalTime = currentRoundLabel(stop).arrivalTime;
            for (const Edge edge : data.transferGraph.edgesFrom(stop)) {
                profiler.countMetric(METRIC_EDGES);
                const int arrivalTime = earliestArrivalTime + data.transferGraph.get(TravelTime, edge);
                const StopId toStop = StopId(data.transferGraph.get(ToVertex, edge));
                AssertMsg(data.isStop(toStop), "Graph contains edges to non stop vertices!");
                arrivalByTransfer(toStop, arrivalTime, stop, earliestArrivalTime);
            }
            stopsUpdatedByTransfer.insert(stop);
        }

        stopsUpdatedByTransfer.sort();
    }

    inline EarliestArrivalLabel& roundLabel(const size_t round, const StopId stop) noexcept
    {
        AssertMsg(data.isStop(stop), "Stop is not really a stop!\n");
        AssertMsg(rounds.size() > round, "Not enough rounds!\n");
        AssertMsg(rounds[round].size() > stop, "Enough rounds, but not enough stops!\n");
        EarliestArrivalLabel& result = rounds[round][stop];
        if (result.timestamp != timestamp) {
            if (round > 0) {
                result.arrivalTime = std::min(result.arrivalTime, roundLabel(round - 1, stop).arrivalTime);
            } else {
                const int distance = initialTransferDistance[stop];
                if (distance != INFTY) {
                    profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
                    result.arrivalTime = sourceDepartureTime + distance;
                    result.parent = sourceStop;
                    result.parentDepartureTime = sourceDepartureTime;
                    result.usesRoute = false;
                }
            }
            result.timestamp = timestamp;
            AssertMsg(result.arrivalTime >= 0, "Negative ArrivalTime!");
        }
        return result;
    }

    inline EarliestArrivalLabel& currentRoundLabel(const StopId stop) noexcept
    {
        AssertMsg(data.isStop(stop), "Stop is not really a stop!\n");
        AssertMsg(roundIndex < rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        return roundLabel(roundIndex, stop);
    }

    inline EarliestArrivalLabel& previousRoundLabel(const StopId stop) noexcept
    {
        AssertMsg(data.isStop(stop), "Stop is not really a stop!\n");
        AssertMsg(roundIndex - 1 < rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        AssertMsg(roundIndex > 0, "Cannot return previous round, because no round exists!");
        return roundLabel(roundIndex - 1, stop);
    }

    inline void startNewRound() noexcept
    {
        AssertMsg(roundIndex + 1 <= rounds.size(), "Round index is out of bounds (roundIndex = " << roundIndex << ", rounds.size() = " << rounds.size() << ")!");
        ++roundIndex;
        if (roundIndex == rounds.size()) {
            if (rounds.empty()) {
                rounds.emplace_back(data.numberOfStops());
            } else {
                rounds.emplace_back(rounds.back());
                if constexpr (TripPruning)
                    lastTrip.emplace_back(lastTrip.back());
            }
        }
    }

    inline bool arrivalByRoute(const StopId stop, const int arrivalTime) noexcept
    {
        AssertMsg(arrivalTime >= 0, "ArrivalTime before the BIG BANG...?!");
        AssertMsg(roundIndex > 0, "arrivalByRoute cannot be used in the first round!");
        AssertMsg(data.isStop(stop), "Stop " << stop << " is out of range!");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "], stop: " << stop << ")!");
        EarliestArrivalLabel& label = currentRoundLabel(stop);

        if (label.arrivalTime <= arrivalTime)
            return false;

        profiler.countMetric(METRIC_STOPS_BY_TRIP);
        label.arrivalTime = arrivalTime;
        stopsUpdatedByRoute.insert(stop);

        // write stop to stopsUpdatedWithTarget
        stopsUpdatedWithTarget[stop] = true;

        return true;
    }

    inline void arrivalByTransfer(const StopId stop, const int arrivalTime, const Vertex parent, const int parentDepartureTime) noexcept
    {
        AssertMsg(arrivalTime >= 0, "ArrivalTime negative?");
        AssertMsg(data.isStop(stop) || stop == targetStop, "Stop " << stop << " is out of range!");
        AssertMsg(arrivalTime >= sourceDepartureTime, "Arriving by route BEFORE departing from the source (source departure time: " << String::secToTime(sourceDepartureTime) << " [" << sourceDepartureTime << "], arrival time: " << String::secToTime(arrivalTime) << " [" << arrivalTime << "])!");
        EarliestArrivalLabel& label = currentRoundLabel(stop);

        if (label.arrivalTime <= arrivalTime)
            return;

        if (data.isStop(stop))
            stopsUpdatedByTransfer.insert(stop);
        profiler.countMetric(METRIC_STOPS_BY_TRANSFER);
        label.arrivalTime = arrivalTime;
        AssertMsg(data.isStop(StopId(parent)), "Parent is not a valid stop!\n");
        label.parent = parent;
        label.parentDepartureTime = parentDepartureTime;
        label.usesRoute = false;
        if constexpr (CollectReachedVertices)
            reachedVertices.insert(stop);
    }

    inline void getJourney(std::vector<JourneyWithStopEvent>& journeys, size_t round, Vertex vertex) noexcept
    {
        AssertMsg(data.isStop(StopId(vertex)), "Stop is not really a stop!\n");
        if (roundLabel(round, StopId(vertex)).arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime))
            return;
        JourneyWithStopEvent journey;
        do {
            AssertMsg(data.isStop(StopId(vertex)), "Stop is not really a stop!\n");
            AssertMsg(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = roundLabel(round, StopId(vertex));
            AssertMsg(label.arrivalTime >= 0, "negative arrTime??");
            journey.emplace_back(label.parent, vertex, label.parentDepartureTime, label.arrivalTime, label.usesRoute, label.routeId);

            vertex = label.parent;
            if (label.usesRoute)
                --round;
        } while (journey.back().from != sourceStop);
        journeys.emplace_back(Vector::reverse(journey));
    }

    inline void getEdgeList(std::vector<JourneyWithStopEvent>& journeys, std::vector<EdgeListOfStopEventIds>& edgeLists, size_t round, Vertex vertex) noexcept
    {
        AssertMsg(data.isStop(StopId(vertex)), "Stop is not really a stop!\n");
        if (roundLabel(round, StopId(vertex)).arrivalTime >= (journeys.empty() ? never : journeys.back().back().arrivalTime))
            return;
        EdgeListOfStopEventIds edgeList;
        JourneyWithStopEvent journey;
        do {
            AssertMsg(data.isStop(StopId(vertex)), "Stop is not really a stop!\nRound Value: " << round << "\n");
            AssertMsg(round != size_t(-1), "Backtracking parent pointers did not pass through the source stop!");
            const EarliestArrivalLabel& label = roundLabel(round, StopId(vertex));
            journey.emplace_back(label.parent, vertex, label.fromStopEventId, label.toStopEventId, label.parentDepartureTime, label.arrivalTime, label.usesRoute, label.routeId);

            AssertMsg(data.isStop(StopId(label.parent)), "Parent " << label.parent << " of vertex " << vertex << " is not really a stop! DepTime Parent: " << label.parentDepartureTime << " ArrTime: " << label.arrivalTime << " UsesRoute: " << label.usesRoute << "\nRound Value: " << round << "\n");

            if (label.usesRoute) {
                edgeList.push_back(std::make_pair(label.fromStopEventId, label.toStopEventId));
                --round;
            }
            vertex = label.parent;
        } while (journey.back().from != sourceStop);

        journeys.emplace_back(Vector::reverse(journey));
        if (edgeList.size() > 0)
            edgeLists.emplace_back(Vector::reverse(edgeList));
    }

    inline void getArrival(std::vector<ArrivalLabel>& labels, size_t round, const Vertex vertex) noexcept
    {
        AssertMsg(data.isStop(StopId(vertex)), "Stop is not really a stop!\n");
        const EarliestArrivalLabel& label = roundLabel(round, StopId(vertex));
        if (label.arrivalTime >= (labels.empty() ? never : labels.back().arrivalTime))
            return;
        labels.emplace_back(label.arrivalTime, round);
    }

    inline void printRoundsForStop(const StopId stop) noexcept
    {
        AssertMsg(data.isStop(stop), stop << " is not a valid stop!");
        std::cout << "Raptor Label for stop " << stop << ":" << std::endl;
        std::cout << std::setw(10) << "Round" << std::setw(14) << "arrivalTime" << std::setw(14) << "parent" << std::setw(14) << "Route" << std::setw(14) << "Run & Timestamp" << std::endl;
        for (size_t i = 0; i <= roundIndex; ++i) {
            EarliestArrivalLabel& label = roundLabel(i, stop);
            std::cout << std::setw(10) << i << std::setw(14) << label.arrivalTime << std::setw(14) << label.parent << std::setw(14) << (int)label.usesRoute << std::setw(14) << (int)label.timestamp << std::endl;
        }
    }

private:
    Data& data;

    std::vector<Round> rounds;
    size_t roundIndex;

    IndexedSet<false, StopId> stopsUpdatedByRoute;
    IndexedSet<false, StopId> stopsUpdatedByTransfer;
    // Stops targetlabel updated
    std::vector<bool> stopsUpdatedWithTarget;

    IndexedMap<StopIndex, false, RouteId> routesServingUpdatedStops;

    StopId sourceStop;
    StopId targetStop; // One-to-one only
    int sourceDepartureTime;
    int timestamp;

    std::vector<int> initialTransferDistance;

    std::vector<std::vector<std::vector<uint16_t>>> lastTrip;

    Profiler profiler;

    IndexedSet<false, Vertex> reachedVertices;
};

}
