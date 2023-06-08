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
#include "../Query/ProfileReachedIndex.h"
#include "../Query/ReachedIndex.h"

namespace TripBased {

struct TripStopIndex {
    TripStopIndex(const TripId trip = noTripId, const StopIndex stopIndex = StopIndex(-1), const int depTime = never)
        : trip(trip)
        , stopIndex(stopIndex)
        , depTime(depTime)
    {
    }

    TripId trip;
    StopIndex stopIndex;
    int depTime;
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

class CalculateARCFlagsProfile {
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

    struct TargetLabel {
        TargetLabel(const long arrivalTime = INFTY, const u_int32_t parent = -1,
            const StopEventId fromStopEventId = noStopEvent, const int run = -1)
            : arrivalTime(arrivalTime)
            , parent(parent)
            , fromStopEventId(fromStopEventId)
            , run(run)
        {
        }

        void clear()
        {
            arrivalTime = INFTY;
            parent = -1;
            fromStopEventId = noStopEvent;
            run = -1;
        }

        long arrivalTime;
        u_int32_t parent;
        StopEventId fromStopEventId;
        int run;
    };

public:
    CalculateARCFlagsProfile(Data& data, std::vector<std::vector<uint8_t>>& uint8Flags,
        std::vector<std::vector<TripStopIndex>>& collectedDepTimes,
        std::vector<TripBased::RouteLabel>& routeLabels)
        : data(data)
        , uint8Flags(uint8Flags)
        , numberOfPartitions(data.raptorData.numberOfPartitions)
        , transferFromSource(data.numberOfStops(), INFTY)
        , lastSource(StopId(0))
        , reachedRoutes(data.numberOfRoutes())
        , queue(data.numberOfStopEvents())
        , queueSize(0)
        , profileReachedIndex(data)
        , runReachedIndex(data)
        , stopsToUpdate(data.numberOfStops())
        , edgeLabels(data.stopEventGraph.numEdges())
        , sourceStop(noStop)
        , collectedDepTimes(collectedDepTimes)
        , tripLabelEdge(data.numberOfStopEvents(), std::make_pair(noEdge, noStopEvent))
        , routeLabels(routeLabels)
        , previousTripLookup(data.numberOfTrips())
        , timestamp(0)
    {
        emptyTargetLabels.assign(16, TargetLabel());
        emptyTargetLabelChanged.assign(16, false);
        emptyMinArrivalTimeFastLookUp.assign(16, INFTY);

        targetLabels.assign(data.raptorData.stopData.size(), emptyTargetLabels);
        targetLabelChanged.assign(data.raptorData.stopData.size(), emptyTargetLabelChanged);
        minArrivalTimeFastLookUp.assign(data.raptorData.stopData.size(), emptyMinArrivalTimeFastLookUp);

        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
        }

        // pre load every previous trip since we need to look this up quite often
        for (const RouteId route : data.routes()) {
            TripId firstTrip = data.firstTripOfRoute[route];
            for (const TripId trip : data.tripsOfRoute(route)) {
                previousTripLookup[trip] = TripId(trip - 1);
            }
            previousTripLookup[firstTrip] = firstTrip;
        }
    }

    inline void run(const Vertex source) noexcept
    {
        run(StopId(source));
    }

    inline void run(const StopId source) noexcept
    {
        sourceStop = source;
        std::vector<RAPTOR::Journey> journeyOfRound;

        // reset everything
        reset();

        computeInitialAndFinalTransfers();

        // perform one EA query from 24:00:00 to get all the journeys only from the
        // second day
        performOneEAQueryAtMidnight();

        // we get the depTimes from the main file that calls this thread
        // collectDepartures();
        size_t i(0), j(0);

        while (i < collectedDepTimes[sourceStop].size()) {
            ++timestamp;

            // clear (without reset)
            clear();

            // now we collect all the trips and stop sequences at a certain
            // timestamp and perform one normal query
            while (j < collectedDepTimes[sourceStop].size() && collectedDepTimes[sourceStop][i].depTime == collectedDepTimes[sourceStop][j].depTime) {
                enqueue(collectedDepTimes[sourceStop][j].trip, StopIndex(collectedDepTimes[sourceStop][j].stopIndex + 1));
                ++j;
            }

            scanTrips();

            // unwind and flag all Journeys
            for (const StopId target : stopsToUpdate) {
                unwindJourneys(target);
            }

            i = j;
        }
    }

    inline void performOneEAQueryAtMidnight() noexcept
    {
        evaluateInitialTransfers();
        scanTrips();
        for (const StopId target : stopsToUpdate)
            unwindJourneys(target);
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
                    --tripIndex;
                    while ((tripIndex > 0) && (label.departureTimes[labelIndex + tripIndex - 1] >= stopDepartureTime)) {
                        --tripIndex;
                    }
                }
                enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1));
                if (tripIndex == 0)
                    break;
            }
        }
    }
    inline void unwindJourneys(const StopId& target) noexcept
    {
        int bestArrivalTime = INFTY;
        int counter(0);
        int partition = data.getPartitionCell(target);
        for (const TargetLabel& label : targetLabels[target]) {
            AssertMsg(counter < 16, "Counter is too high!\n");
            if ((!targetLabelChanged[target][counter++]) || label.arrivalTime >= bestArrivalTime)
                continue;
            bestArrivalTime = label.arrivalTime;
            getJourneyAndUnwind(label, partition, target);
        }
    }

private:
    inline void reset() noexcept
    {
        profileReachedIndex.clear();
        minArrivalTimeFastLookUp.assign(data.raptorData.stopData.size(), emptyMinArrivalTimeFastLookUp);
        targetLabels.assign(data.raptorData.stopData.size(), emptyTargetLabels);
        clear();
    }

    inline void clear() noexcept
    {
        queueSize = 0;
        runReachedIndex.clear();
        targetLabelChanged.assign(data.raptorData.stopData.size(), emptyTargetLabelChanged);
        stopsToUpdate.clear();
    }

    inline void computeInitialAndFinalTransfers() noexcept
    {
        transferFromSource[lastSource] = INFTY;
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(lastSource)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            transferFromSource[stop] = INFTY;
        }
        transferFromSource[sourceStop] = 0;
        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(sourceStop)) {
            const Vertex stop = data.raptorData.transferGraph.get(ToVertex, edge);
            transferFromSource[stop] = data.raptorData.transferGraph.get(TravelTime, edge);
        }
        lastSource = sourceStop;
    }

    inline void scanTrips() noexcept
    {
        size_t roundBegin = 0;
        size_t roundEnd = queueSize;
        u_int8_t n = 1;
        int travelTime(-1);
        StopId stop(-1);
        Vertex transferStop(-1);

        while (roundBegin < roundEnd && n < 16) {
            // Evaluate final transfers in order to check if the target is
            // reachable
            for (size_t i = roundBegin; i < roundEnd; ++i) {
                const TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; ++j) {
                    stop = data.arrivalEvents[j].stop;
                    if (addTargetLabel(stop, data.arrivalEvents[j].arrivalTime, i, n, j)) {
                        AssertMsg(data.raptorData.transferGraph.isVertex(stop),
                            "This stop is not represented in the transfergraph!\n");
                        for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
                            transferStop = data.raptorData.transferGraph.get(ToVertex, edge);
                            if (!data.isStop(transferStop))
                                continue;
                            travelTime = data.raptorData.transferGraph.get(TravelTime, edge);
                            addTargetLabel(StopId(transferStop), data.arrivalEvents[j].arrivalTime + travelTime, i, n, j);
                        }
                    }
                }
            }
            for (size_t i = roundBegin; i < roundEnd; ++i) {
                TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; ++j) {
                    if (data.arrivalEvents[j].arrivalTime > minArrivalTimeFastLookUp[data.arrivalEvents[j].stop][n])
                        continue;
                    for (Edge edge : data.stopEventGraph.edgesFrom(Vertex(j))) {
                        enqueue(edge, i, n, j);
                    }
                }
            }
            roundBegin = roundEnd;
            roundEnd = queueSize;

            ++n;
        }
    }

    /*
    inline TripId getPreviousTripInRoute(const TripId trip) noexcept
    {
        const TripId firstTripId = data.firstTripOfRoute[data.routeOfTrip[trip]];

        AssertMsg(firstTripId <= trip, "We found a trip that is earlier than the first trip of the same route!\n");
        if (trip > firstTripId)
            return TripId(trip - 1);
        return firstTripId;
    }
    */

    inline bool discard(const TripId trip, const StopIndex index, const int n = 1)
    {
        if (runReachedIndex.alreadyReached(trip, index)) [[likely]]
            return true;
        if (profileReachedIndex(trip, 1) < index) [[likely]]
            return true;
        if (n > 1 && profileReachedIndex.alreadyReached(trip, index, n)) [[likely]]
            return true;
        const TripId prevTrip = previousTripLookup[trip];
        return (prevTrip != trip && profileReachedIndex.alreadyReached(prevTrip, index, n + 1));
    }

    inline void enqueue(const TripId trip, const StopIndex index) noexcept
    {
        if (discard(trip, index))
            return;
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        queue[queueSize] = TripLabel(StopEventId(firstEvent + index), StopEventId(firstEvent + runReachedIndex(trip)));
        tripLabelEdge[queueSize] = std::make_pair(noEdge, noStopEvent); // noStopEvent since this is the first trip
        ++queueSize;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");

        runReachedIndex.update(trip, index);
        profileReachedIndex.update(trip, index, 1);
    }

    inline void enqueue(const Edge edge, const size_t parent, const u_int8_t n,
        const StopEventId fromStopEventId) noexcept
    {
        const EdgeLabel& label = edgeLabels[edge];
        if (discard(label.trip, StopIndex(label.stopEvent - label.firstEvent), n))
            return;
        queue[queueSize] = TripLabel(label.stopEvent, StopEventId(label.firstEvent + runReachedIndex(label.trip)), parent);
        tripLabelEdge[queueSize] = std::make_pair(edge, fromStopEventId);
        ++queueSize;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");

        runReachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent));
        profileReachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent), n + 1);
    }

    inline bool addTargetLabel(const StopId stop, const int newArrivalTime, const u_int32_t parent = -1,
        const u_int8_t n = 0, const StopEventId fromStopEventId = noStopEvent) noexcept
    {
        // discard *ONLY* if there is no improvement or if same arrival time but we already found it this round
        if (newArrivalTime > minArrivalTimeFastLookUp[stop][n])
            return false;
        if (newArrivalTime == minArrivalTimeFastLookUp[stop][n] && targetLabels[stop][n].run == timestamp)
            return false;

        targetLabels[stop][n].arrivalTime = newArrivalTime;
        targetLabels[stop][n].parent = parent;
        targetLabels[stop][n].fromStopEventId = fromStopEventId;
        targetLabels[stop][n].run = timestamp;

        targetLabelChanged[stop][n] = true;

        stopsToUpdate.insert(stop);

        minArrivalTimeFastLookUp[stop][n] = newArrivalTime;

#pragma omp simd
        for (int i = n + 1; i < 16; ++i) {
            if (targetLabels[stop][i].arrivalTime > targetLabels[stop][n].arrivalTime) {
                targetLabels[stop][i].clear();
            }
            if (minArrivalTimeFastLookUp[stop][i] > newArrivalTime)
                minArrivalTimeFastLookUp[stop][i] = newArrivalTime;
        }
        return true;
    }

    inline void getJourneyAndUnwind(const TargetLabel& targetLabel, const int targetCell, const StopId target) noexcept
    {
        u_int32_t parent = targetLabel.parent;
        if (parent == u_int32_t(-1))
            return;

        StopEventId departureStopEvent = noStopEvent;
        Vertex departureStop = target;

        while (parent != u_int32_t(-1)) {
            AssertMsg(parent < queueSize, "Parent " << parent << " is out of range!");
            const TripLabel& label = queue[parent];
            StopEventId arrivalStopEvent;
            Edge edge;
            std::tie(edge, arrivalStopEvent) = (departureStopEvent == noStopEvent)
                ? std::make_pair(noEdge, targetLabel.fromStopEventId)
                : getParent(label, StopEventId(departureStopEvent + 1));

            AssertMsg(arrivalStopEvent != noStopEvent, "arrivalStopEvent is null");

            departureStopEvent = StopEventId(label.begin - 1);
            departureStop = data.getStopOfStopEvent(departureStopEvent);

            parent = label.parent;
            if (edge != noEdge) {
                uint8Flags[edge][targetCell] = 1;
            }
        }
    }

    inline std::pair<Edge, StopEventId> getParent(const TripLabel& parentLabel,
        const StopEventId departureStopEvent) const noexcept
    {
        for (StopEventId i = parentLabel.begin; i < parentLabel.end; ++i) {
            for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(i))) {
                if (edgeLabels[edge].stopEvent == departureStopEvent) {
                    return std::make_pair(edge, i);
                }
            }
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noEdge, noStopEvent);
    }

private:
    Data& data;

    std::vector<std::vector<uint8_t>>& uint8Flags;

    int numberOfPartitions;
    std::vector<int> transferFromSource;
    StopId lastSource;

    IndexedSet<false, RouteId> reachedRoutes;

    std::vector<TripLabel> queue;
    size_t queueSize;
    ProfileReachedIndex profileReachedIndex;
    ReachedIndex runReachedIndex;

    // for every stop
    std::vector<std::vector<TargetLabel>> targetLabels;
    std::vector<std::vector<bool>> targetLabelChanged;
    std::vector<std::vector<long>> minArrivalTimeFastLookUp;
    std::vector<TargetLabel> emptyTargetLabels;
    std::vector<bool> emptyTargetLabelChanged;
    std::vector<long> emptyMinArrivalTimeFastLookUp;

    // check which stops need to be updated during one round
    IndexedSet<false, StopId> stopsToUpdate;

    std::vector<EdgeLabel> edgeLabels;

    StopId sourceStop;

    std::vector<std::vector<TripStopIndex>>& collectedDepTimes;

    // this is to get the edge and the fromStopEventId  of a triplabel faster
    std::vector<std::pair<Edge, StopEventId>> tripLabelEdge;

    std::vector<TripBased::RouteLabel>& routeLabels;
    std::vector<TripId> previousTripLookup;

    int timestamp;
};

} // namespace TripBased
