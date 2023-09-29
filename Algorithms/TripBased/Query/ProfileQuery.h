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

#include "../../../DataStructures/Container/Set.h"
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/RAPTOR/Entities/RouteSegment.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/String/String.h"
#include "ProfileReachedIndex.h"
#include "Profiler.h"

namespace TripBased {

template <typename PROFILER = NoProfiler>
class ProfileQuery {
public:
    using Profiler = PROFILER;
    using Type = ProfileQuery<Profiler>;

private:
    struct TripLabel {
        TripLabel(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent,
            const u_int32_t parent = -1)
            : begin(begin)
            , end(end)
            , parent(parent)
        {
        }
        StopEventId begin;
        StopEventId end;
        u_int32_t parent;
    };

    struct EdgeRange {
        EdgeRange()
            : begin(noEdge)
            , end(noEdge)
        {
        }

        Edge begin;
        Edge end;
    };

    struct EdgeLabel {
        EdgeLabel(const StopEventId stopEvent = noStopEvent, const TripId trip = noTripId,
            const StopEventId firstEvent = noStopEvent)
            : stopEvent(stopEvent)
            , trip(trip)
            , firstEvent(firstEvent)
        {
        }
        StopEventId stopEvent;
        TripId trip;
        StopEventId firstEvent;
    };

    struct RouteLabel {
        RouteLabel()
            : numberOfTrips(0)
        {
        }
        inline StopIndex end() const noexcept
        {
            return StopIndex(departureTimes.size() / numberOfTrips);
        }
        u_int32_t numberOfTrips;
        std::vector<int> departureTimes;
    };

    struct TargetLabel {
        TargetLabel(const int arrivalTime = INFTY, const u_int32_t parent = -1)
            : arrivalTime(arrivalTime)
            , parent(parent)
        {
        }

        void clear()
        {
            arrivalTime = INFTY;
            parent = -1;
        }

        int arrivalTime;
        u_int32_t parent;
    };

    struct TripStopIndex {
        TripStopIndex(const TripId trip = noTripId, const StopIndex stopIndex = StopIndex(-1),
            const int depTime = never)
            : trip(trip)
            , stopIndex(stopIndex)
            , depTime(depTime)
        {
        }

        TripId trip;
        StopIndex stopIndex;
        int depTime;
    };

public:
    ProfileQuery(const Data& data)
        : data(data)
        , reverseTransferGraph(data.raptorData.transferGraph)
        , transferFromSource(data.numberOfStops(), INFTY)
        , transferToTarget(data.numberOfStops(), INFTY)
        , lastSource(StopId(0))
        , lastTarget(StopId(0))
        , reachedRoutes(data.numberOfRoutes())
        , queue(data.numberOfStopEvents())
        , edgeRanges(data.numberOfStopEvents())
        , queueSize(0)
        , reachedIndex(data)
        , targetLabels(1)
        , minArrivalTimeFastLookUp(16, INFTY)
        , edgeLabels(data.stopEventGraph.numEdges())
        , sourceStop(noStop)
        , targetStop(noStop)
        , minDepartureTime(never)
        , maxDepartureTime(never)
        , targetLabelChanged(16, false)
        , routeLabels(data.stopEventGraph.numEdges())
    {
        collectedDepTimes.reserve(data.raptorData.numberOfTrips()); // can be adjusted
        allJourneys.reserve(32);
        reverseTransferGraph.revert();
        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
        }
        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const RAPTOR::StopEvent* stopEvents = data.raptorData.firstTripOfRoute(route);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; trip++) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] = stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }

        profiler.registerPhases({ PHASE_SCAN_INITIAL, PHASE_COLLECT_DEPTIMES, PHASE_SCAN_TRIPS });
        profiler.registerMetrics({ METRIC_ROUNDS, METRIC_SCANNED_TRIPS, METRIC_SCANNED_STOPS, METRIC_RELAXED_TRANSFERS,
            METRIC_ENQUEUES, METRIC_ADD_JOURNEYS });
    }

    inline void run(const Vertex source, const Vertex target, const int minDepartureTime,
        const int maxDepartureTime) noexcept
    {
        AssertMsg(data.isStop(source), "Source " << source << " is not a stop!");
        AssertMsg(data.isStop(target), "Target " << target << " is not a stop!");
        AssertMsg(minDepartureTime <= maxDepartureTime,
            "Minimum Departure Time needs to smaller or equal to the Maximum "
            "Departure Time!");
        run(StopId(source), StopId(target), minDepartureTime, maxDepartureTime);
    }

    inline void run(const StopId source, const StopId target, const int minDepTime, const int maxDepTime) noexcept
    {
        profiler.start();
        sourceStop = source;
        targetStop = target;
        minDepartureTime = minDepTime;
        maxDepartureTime = maxDepTime;
        std::vector<RAPTOR::Journey> journeyOfRound;

        // clear everything
        clear();
        computeInitialAndFinalTransfers();
        evaluateInitialTransfers();
        scanTrips();
        journeyOfRound = getJourneys();
        allJourneys.insert(allJourneys.end(), journeyOfRound.begin(), journeyOfRound.end());
        targetLabelChanged.assign(16, false);

        collectDepartures();
        // note: this vector is not duplicate free
        size_t i(0), j(0);
        while (i < collectedDepTimes.size()) {
            // perform one "normal" query
            queueSize = 0;
            while (j < collectedDepTimes.size() && collectedDepTimes[i].depTime == collectedDepTimes[j].depTime) {
                enqueue(collectedDepTimes[j].trip, StopIndex(collectedDepTimes[j].stopIndex + 1));
                ++j;
            }
            scanTrips();
            journeyOfRound = getJourneys();
            allJourneys.insert(allJourneys.end(), journeyOfRound.begin(), journeyOfRound.end());
            i = j;
            targetLabelChanged.assign(16, false);
        }
        profiler.done();
    }

    inline void evaluateInitialTransfers() noexcept
    {
        reachedRoutes.clear();
        for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(sourceStop)) {
            reachedRoutes.insert(route.routeId);
        }
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(sourceStop)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(StopId(stop))) {
                reachedRoutes.insert(route.routeId);
            }
        }
        reachedRoutes.sort();
        for (const RouteId route : reachedRoutes) {
            const RouteLabel& label = routeLabels[route];
            const StopIndex endIndex = label.end();
            const TripId firstTrip = data.firstTripOfRoute[route];
            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            TripId tripIndex = noTripId;
            for (StopIndex stopIndex(0); stopIndex < endIndex; stopIndex++) {
                const int timeFromSource = transferFromSource[stops[stopIndex]];
                if (timeFromSource == INFTY)
                    continue;
                const int stopDepartureTime = 24 * 60 * 60 + timeFromSource;
                const u_int32_t labelIndex = stopIndex * label.numberOfTrips;
                if (tripIndex >= label.numberOfTrips) {
                    tripIndex = std::lower_bound(TripId(0), TripId(label.numberOfTrips), stopDepartureTime,
                        [&](const TripId trip, const int time) {
                            return label.departureTimes[labelIndex + trip] < time;
                        });
                    if (tripIndex >= label.numberOfTrips)
                        continue;
                } else {
                    if (label.departureTimes[labelIndex + tripIndex - 1] < stopDepartureTime)
                        continue;
                    tripIndex--;
                    while ((tripIndex > 0) && (label.departureTimes[labelIndex + tripIndex - 1] >= stopDepartureTime)) {
                        tripIndex--;
                    }
                }
                enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1));
                if (tripIndex == 0)
                    break;
            }
        }
    }

    inline Profiler& getProfiler() noexcept
    {
        return profiler;
    }

    const std::vector<TripStopIndex>& getCollectedDepTimes() noexcept
    {
        return collectedDepTimes;
    }

    inline std::vector<RAPTOR::Journey> getAllJourneys() const noexcept
    {
        return allJourneys;
    }

    inline std::vector<RAPTOR::Journey> getJourneys() const noexcept
    {
        std::vector<RAPTOR::Journey> result;
        int bestArrivalTime = INFTY;
        int counter(0);
        for (const TargetLabel& label : targetLabels) {
            if ((!targetLabelChanged[counter++]) || label.arrivalTime >= bestArrivalTime)
                continue;
            bestArrivalTime = label.arrivalTime;
            result.emplace_back(getJourney(label));
        }
        return result;
    }

private:
    inline void clear() noexcept
    {
        queueSize = 0;
        reachedIndex.clear();
        targetLabels.assign(16, TargetLabel());
        minArrivalTimeFastLookUp.assign(16, INFTY);
        allJourneys.clear();
        targetLabelChanged.assign(16, false);
    }

    inline void computeInitialAndFinalTransfers() noexcept
    {
        profiler.startPhase();
        transferFromSource[lastSource] = INFTY;
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(lastSource)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            transferFromSource[stop] = INFTY;
        }
        transferToTarget[lastTarget] = INFTY;
        for (const Edge edge : reverseTransferGraph.edgesFrom(lastTarget)) {
            const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
            transferToTarget[stop] = INFTY;
        }
        transferFromSource[sourceStop] = 0;
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(sourceStop)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            transferFromSource[stop] = data.raptorData.transferGraph.get(TravelTime, edge);
        }
        transferToTarget[targetStop] = 0;
        if (sourceStop == targetStop)
            addTargetLabel(minDepartureTime);
        for (const Edge edge : reverseTransferGraph.edgesFrom(targetStop)) {
            const Vertex stop = reverseTransferGraph.get(ToVertex, edge);
            if (stop == sourceStop)
                addTargetLabel(minDepartureTime + reverseTransferGraph.get(TravelTime, edge));
            transferToTarget[stop] = reverseTransferGraph.get(TravelTime, edge);
        }
        lastSource = sourceStop;
        lastTarget = targetStop;
        profiler.donePhase(PHASE_SCAN_INITIAL);
    }

    inline void collectDepartures() noexcept
    {
        profiler.startPhase();
        collectedDepTimes.clear();
        // get all reachable routes (meaning also by footpaths)
        reachedRoutes.clear();
        for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(sourceStop)) {
            reachedRoutes.insert(route.routeId);
        }
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(sourceStop)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            for (const RAPTOR::RouteSegment& route : data.raptorData.routesContainingStop(StopId(stop))) {
                reachedRoutes.insert(route.routeId);
            }
        }
        reachedRoutes.sort();
        for (const RouteId route : reachedRoutes) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            const RAPTOR::StopEvent* stopEvents = data.raptorData.firstTripOfRoute(route);
            const TripId firstTrip = data.firstTripOfRoute[route];
            for (size_t stopEventIndex = 0; stopEventIndex < data.raptorData.numberOfStopEventsInRoute(route);
                 ++stopEventIndex) {
                if ((stopEventIndex + 1) % numberOfStops == 0)
                    continue;
                const StopId stop = stops[stopEventIndex % numberOfStops];
                const int walkingTime = transferFromSource[stop];
                if (walkingTime == INFTY)
                    continue;
                const int departureTime = stopEvents[stopEventIndex].departureTime - walkingTime;
                if (departureTime < minDepartureTime || departureTime >= maxDepartureTime)
                    continue;
                collectedDepTimes.push_back(TripStopIndex(TripId(firstTrip + (int)(stopEventIndex / numberOfStops)),
                    StopIndex(stopEventIndex % numberOfStops), departureTime));
            }
        }
        // sort collectedDepTimes desc
        std::stable_sort(collectedDepTimes.begin(), collectedDepTimes.end(),
            [](const TripStopIndex a, const TripStopIndex b) { return a.depTime > b.depTime; });
        profiler.donePhase(PHASE_COLLECT_DEPTIMES);
    }

    inline void scanTrips() noexcept
    {
        profiler.startPhase();
        size_t roundBegin = 0;
        size_t roundEnd = queueSize;
        u_int8_t n = 1;
        while (roundBegin < roundEnd && n < 16) {
            profiler.countMetric(METRIC_ROUNDS);
            // Evaluate final transfers in order to check if the target is
            // reachable
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const TripLabel& label = queue[i];
                profiler.countMetric(METRIC_SCANNED_TRIPS);
                for (StopEventId j = label.begin; j < label.end; j++) {
                    profiler.countMetric(METRIC_SCANNED_STOPS);
                    // change the order back
                    const int timeToTarget = transferToTarget[data.arrivalEvents[j].stop];
                    if (data.arrivalEvents[j].arrivalTime >= minArrivalTimeFastLookUp[n])
                        break;
                    if (timeToTarget != INFTY) {
                        addTargetLabel(data.arrivalEvents[j].arrivalTime + timeToTarget, i, n);
                    }
                }
            }
            // Find the range of transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; j++) {
                    if (data.arrivalEvents[j].arrivalTime >= minArrivalTimeFastLookUp[n])
                        label.end = j;
                }
                edgeRanges[i].begin = data.stopEventGraph.beginEdgeFrom(Vertex(label.begin));
                edgeRanges[i].end = data.stopEventGraph.beginEdgeFrom(Vertex(label.end));
            }
            // Relax the transfers for each trip
            for (size_t i = roundBegin; i < roundEnd; i++) {
                const EdgeRange& label = edgeRanges[i];
                for (Edge edge = label.begin; edge < label.end; edge++) {
                    profiler.countMetric(METRIC_RELAXED_TRANSFERS);
                    enqueue(edge, i, n);
                }
            }
            roundBegin = roundEnd;
            roundEnd = queueSize;

            ++n;
        }
        profiler.donePhase(PHASE_SCAN_TRIPS);
    }

    inline void enqueue(const TripId trip, const StopIndex index) noexcept
    {
        profiler.countMetric(METRIC_ENQUEUES);
        if (reachedIndex.alreadyReached(trip, index, 1))
            return;
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        queue[queueSize] = TripLabel(StopEventId(firstEvent + index), StopEventId(firstEvent + reachedIndex(trip, 1)));
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(trip, index, 1);
    }

    inline void enqueue(const Edge edge, const size_t parent, const u_int8_t n) noexcept
    {
        profiler.countMetric(METRIC_ENQUEUES);
        const EdgeLabel& label = edgeLabels[edge];
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent, n + 1))
            return;
        queue[queueSize] = TripLabel(label.stopEvent, StopEventId(label.firstEvent + reachedIndex(label.trip, n + 1)), parent);
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent), n + 1);
    }

    inline void addTargetLabel(const int newArrivalTime, const u_int32_t parent = -1, const u_int8_t n = 0) noexcept
    {
        profiler.countMetric(METRIC_ADD_JOURNEYS);
        if (newArrivalTime < minArrivalTimeFastLookUp[n]) {
            targetLabels[n].arrivalTime = newArrivalTime;
            targetLabels[n].parent = parent;

            targetLabelChanged[n] = true;

            minArrivalTimeFastLookUp[n] = newArrivalTime;
// std::fill(minArrivalTimeFastLookUp.begin() + n,
// minArrivalTimeFastLookUp.end(), newArrivalTime);

// do i need to remove everything with greater n and greater arrivaltime? =>
// yes, i think
#pragma omp simd
            for (int i = n + 1; i < 16; ++i) {
                if (targetLabels[i].arrivalTime > targetLabels[n].arrivalTime) {
                    targetLabels[i].clear();
                }
                if (minArrivalTimeFastLookUp[i] > newArrivalTime) {
                    minArrivalTimeFastLookUp[i] = newArrivalTime;
                }
            }
        }
    }

    inline RAPTOR::Journey getJourney(const TargetLabel& targetLabel) const noexcept
    {
        RAPTOR::Journey result;
        u_int32_t parent = targetLabel.parent;
        if (parent == u_int32_t(-1)) {
            result.emplace_back(sourceStop, targetStop, minDepartureTime, targetLabel.arrivalTime, false);
            return result;
        }
        StopEventId departureStopEvent = noStopEvent;
        Vertex departureStop = targetStop;
        int lastTime(minDepartureTime);
        while (parent != u_int32_t(-1)) {
            AssertMsg(parent < queueSize, "Parent " << parent << " is out of range!");
            const TripLabel& label = queue[parent];
            StopEventId arrivalStopEvent;
            Edge edge;
            std::tie(arrivalStopEvent, edge) = (departureStopEvent == noStopEvent)
                ? getParent(label, targetLabel)
                : getParent(label, StopEventId(departureStopEvent + 1));

            const StopId arrivalStop = data.getStopOfStopEvent(arrivalStopEvent);
            const int arrivalTime = data.raptorData.stopEvents[arrivalStopEvent].arrivalTime;
            const int transferArrivalTime = (edge == noEdge) ? targetLabel.arrivalTime : arrivalTime + data.stopEventGraph.get(TravelTime, edge);
            result.emplace_back(arrivalStop, departureStop, arrivalTime, transferArrivalTime, edge);

            departureStopEvent = StopEventId(label.begin - 1);
            departureStop = data.getStopOfStopEvent(departureStopEvent);
            const RouteId route = data.getRouteOfStopEvent(departureStopEvent);
            const int departureTime = data.raptorData.stopEvents[departureStopEvent].departureTime;
            lastTime = departureTime;
            result.emplace_back(departureStop, arrivalStop, departureTime, arrivalTime, true, route);

            parent = label.parent;
        }
        const int timeFromSource = transferFromSource[departureStop];
        result.emplace_back(sourceStop, departureStop, lastTime - timeFromSource, lastTime, noEdge);
        Vector::reverse(result);
        return result;
    }

    inline std::pair<StopEventId, Edge> getParent(const TripLabel& parentLabel,
        const StopEventId departureStopEvent) const noexcept
    {
        for (StopEventId i = parentLabel.begin; i < parentLabel.end; i++) {
            for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(i))) {
                if (edgeLabels[edge].stopEvent == departureStopEvent)
                    return std::make_pair(i, edge);
            }
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

    inline std::pair<StopEventId, Edge> getParent(const TripLabel& parentLabel,
        const TargetLabel& targetLabel) const noexcept
    {
        // Final transfer to target may start exactly at parentLabel.end if it has
        // length 0
        const TripId trip = data.tripOfStopEvent[parentLabel.begin];
        const StopEventId end = data.firstStopEventOfTrip[trip + 1];
        for (StopEventId i = parentLabel.begin; i < end; i++) {
            const int timeToTarget = transferToTarget[data.arrivalEvents[i].stop];
            if (timeToTarget == INFTY)
                continue;
            if (data.arrivalEvents[i].arrivalTime + timeToTarget == targetLabel.arrivalTime)
                return std::make_pair(i, noEdge);
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noStopEvent, noEdge);
    }

private:
    const Data& data;

    TransferGraph reverseTransferGraph;
    std::vector<int> transferFromSource;
    std::vector<int> transferToTarget;
    StopId lastSource;
    StopId lastTarget;

    IndexedSet<false, RouteId> reachedRoutes;

    std::vector<TripLabel> queue;
    std::vector<EdgeRange> edgeRanges;
    size_t queueSize;
    ProfileReachedIndex reachedIndex;

    std::vector<TargetLabel> targetLabels;
    std::vector<int> minArrivalTimeFastLookUp;

    std::vector<EdgeLabel> edgeLabels;

    StopId sourceStop;
    StopId targetStop;
    int minDepartureTime;
    int maxDepartureTime;

    std::vector<TripStopIndex> collectedDepTimes;
    std::vector<RAPTOR::Journey> allJourneys;
    std::vector<bool> targetLabelChanged;

    std::vector<RouteLabel> routeLabels;
    Profiler profiler;
};

} // namespace TripBased
