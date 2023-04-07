/**********************************************************************************
 Copyright (c) 2022 Patrick Steil
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

#include <unordered_map>
#include "../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../DataStructures/RAPTOR/Entities/RouteSegment.h"
#include "../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/Container/Set.h"
#include "../../../Helpers/String/String.h"

#include "../Query/ProfileReachedIndex.h"

namespace TripBased {

struct BufferJourneyLeg {
    BufferJourneyLeg(const TripId tripId = noTripId, const StopId start = StopId(0), const StopId end = StopId(0)) :
        tripId(tripId),
        start(start),
        end(end) {
    }

    TripId tripId;
    StopId start;
    StopId end;
};

struct TripStopIndex {
    TripStopIndex(const TripId trip = noTripId, const StopIndex stopIndex = StopIndex(-1), const int depTime = never) :
        trip(trip),
        stopIndex(stopIndex),
        depTime(depTime) {
    }

    TripId trip;
    StopIndex stopIndex;
    int depTime;
};

struct RouteLabel {
    RouteLabel() :
        numberOfTrips(0) {
    }
    inline StopIndex end() const noexcept {
        return StopIndex(departureTimes.size() / numberOfTrips);
    }
    u_int32_t numberOfTrips;
    std::vector<int> departureTimes;
};


class CalculateARCFlagsProfile {

private:
    struct TripLabel {
        TripLabel(const StopEventId begin = noStopEvent, const StopEventId end = noStopEvent, const u_int32_t parent = -1) :
            begin(begin),
            end(end),
            parent(parent) {
        }
        StopEventId begin;
        StopEventId end;
        u_int32_t parent;
    };

    struct EdgeRange {
        EdgeRange() : begin(noEdge), end(noEdge) {}

        Edge begin;
        Edge end;
    };

    struct EdgeLabel {
        EdgeLabel(const StopEventId stopEvent = noStopEvent, const TripId trip = noTripId, const StopEventId firstEvent = noStopEvent) :
            stopEvent(stopEvent),
            trip(trip),
            firstEvent(firstEvent) {
        }
        StopEventId stopEvent;
        TripId trip;
        StopEventId firstEvent;
    };

    struct TargetLabel {
        TargetLabel(const long arrivalTime = INFTY, const u_int32_t parent = -1, const StopEventId fromStopEventId = noStopEvent) :
            arrivalTime(arrivalTime),
            parent(parent),
            fromStopEventId(fromStopEventId) {
        }

        void clear() {
            arrivalTime = INFTY;
            parent = -1;
            fromStopEventId = noStopEvent;
        }

        long arrivalTime;
        u_int32_t parent;
        StopEventId fromStopEventId;
    };

public:
    CalculateARCFlagsProfile(Data& data, std::vector<std::vector<bool>>& flagsOfThread, std::vector<std::vector<TripStopIndex>>& collectedDepTimes, int minDepartureTime, int maxDepartureTime, std::vector<TripBased::RouteLabel>& routeLabels) :
        data(data),
        flagsOfThread(flagsOfThread),
        numberOfPartitions(data.raptorData.numberOfPartitions),
        transferFromSource(data.numberOfStops(), INFTY),
        lastSource(StopId(0)),
        reachedRoutes(data.numberOfRoutes()),
        queue(data.numberOfStopEvents()),
        queueSize(0),
        reachedIndex(data),
        stopsToUpdate(data.numberOfStops()),
        edgeLabels(data.stopEventGraph.numEdges()),
        sourceStop(noStop),
        minDepartureTime(minDepartureTime),
        maxDepartureTime(maxDepartureTime),
        collectedDepTimes(collectedDepTimes),
        tripLabelEdge(data.numberOfStopEvents(), std::make_pair(noEdge, noStopEvent)),
        routeLabels(routeLabels) {
        // init empty version
        emptyTargetLabels.assign(16, TargetLabel());
        emptyTargetLabelChanged.assign(16, false);
        emptyMinArrivalTimeFastLookUp.assign(16, INFTY);
        emptyBufferJourneyForOneStop.assign(16, {}); // init with 16 empty vector

        targetLabels.assign(data.raptorData.stopData.size(), emptyTargetLabels);
        targetLabelChanged.assign(data.raptorData.stopData.size(), emptyTargetLabelChanged);
        minArrivalTimeFastLookUp.assign(data.raptorData.stopData.size(), emptyMinArrivalTimeFastLookUp);
        bufferedJourneys.assign(data.numberOfStops(), emptyBufferJourneyForOneStop);

        for (const Edge edge : data.stopEventGraph.edges()) {
            edgeLabels[edge].stopEvent = StopEventId(data.stopEventGraph.get(ToVertex, edge) + 1);
            edgeLabels[edge].trip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
            edgeLabels[edge].firstEvent = data.firstStopEventOfTrip[edgeLabels[edge].trip];
        }
    }

    inline void run(const Vertex source, const bool buffer = true) noexcept {
        run(StopId(source), minDepartureTime, maxDepartureTime, buffer);
    }

    inline void run(const StopId source, const int minDepTime, const int maxDepTime, const bool buffer = true) noexcept {
        sourceStop = source;
        minDepartureTime = minDepTime;
        maxDepartureTime = maxDepTime;
        std::vector<RAPTOR::Journey> journeyOfRound;

        // clear everything
        clear();

        computeInitialAndFinalTransfers();

        // perform one EA query from 24:00:00 to get all the journeys only from the second day
        performOneEAQueryAtMidnight();

        // we get the depTimes from the main file that calls this thread
        // collectDepartures();
        // note: vector is not duplicate free
        size_t i(0), j(0);
        while (i < collectedDepTimes[source].size()) {
            stopsToUpdate.clear();
            // now we collect all the trips and stop sequences at a certain timestamp and perform one normal query
            // THIS IS IMPORTANT
            queueSize = 0;
            while (j < collectedDepTimes[source].size() && collectedDepTimes[source][i].depTime == collectedDepTimes[source][j].depTime) {
                enqueue(collectedDepTimes[source][j].trip, StopIndex(collectedDepTimes[source][j].stopIndex + 1));
                ++j;
            }
            // start BFS phase
            scanTrips();
            for (const StopId target : stopsToUpdate) {
                if (buffer) emptyBuffer(target, collectedDepTimes[source][j-1].depTime, target);
                unwindJourneys(target, buffer);
            }
            // update and go to next timestamp
            i = j;
            // this is needed to check which targetLabels have changed
            targetLabelChanged.assign(data.raptorData.stopData.size(), emptyTargetLabelChanged);
        }
        if (buffer) {
            for (StopId target : data.stops()) {
                emptyBuffer(target, 0, target, false);
            }
        }
    }


    inline void performOneEAQueryAtMidnight() noexcept {
        evaluateInitialTransfers();
        scanTrips();
        for (const StopId target : stopsToUpdate) {
            unwindJourneys(target);
        }
        targetLabelChanged.assign(data.raptorData.stopData.size(), emptyTargetLabelChanged);
        stopsToUpdate.clear();
    }
    
    inline void evaluateInitialTransfers() noexcept {
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
                if (timeFromSource == INFTY) continue;
                const int stopDepartureTime = 24 * 60 * 60 + timeFromSource;
                const u_int32_t labelIndex = stopIndex * label.numberOfTrips;
                if (tripIndex >= label.numberOfTrips) {
                    tripIndex = std::lower_bound(TripId(0), TripId(label.numberOfTrips), stopDepartureTime, [&](const TripId trip, const int time) {
                        return label.departureTimes[labelIndex + trip] < time;
                    });
                    if (tripIndex >= label.numberOfTrips) continue;
                } else {
                    if (label.departureTimes[labelIndex + tripIndex - 1] < stopDepartureTime) continue;
                    tripIndex--;
                    while ((tripIndex > 0) && (label.departureTimes[labelIndex + tripIndex - 1] >= stopDepartureTime)) {
                        tripIndex--;
                    }
                }
                enqueue(firstTrip + tripIndex, StopIndex(stopIndex + 1));
                if (tripIndex == 0) break;
            }
        }
    }
    inline void unwindJourneys(const StopId& target, const bool buffer = true) noexcept {
        int bestArrivalTime = INFTY;
        int counter(0);
        int partition = data.getPartitionCell(target);
        for (const TargetLabel& label : targetLabels[target]) {
            AssertMsg(counter < 16, "Counter is too high!\n");
            if ((!targetLabelChanged[target][counter++]) || label.arrivalTime >= bestArrivalTime) continue;
            bestArrivalTime = label.arrivalTime;
            getJourneyAndUnwind(label, partition, target, counter-1, buffer);
        }
    }

private:
    inline void clear() noexcept {
        queueSize = 0;
        reachedIndex.clear();
        targetLabels.assign(data.raptorData.stopData.size(), emptyTargetLabels);
        minArrivalTimeFastLookUp.assign(data.raptorData.stopData.size(), emptyMinArrivalTimeFastLookUp);
        targetLabelChanged.assign(data.raptorData.stopData.size(), emptyTargetLabelChanged);
        stopsToUpdate.clear();
    }

    inline void computeInitialAndFinalTransfers() noexcept {
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


    inline void scanTrips() noexcept {
        size_t roundBegin = 0;
        size_t roundEnd = queueSize;
        u_int8_t n = 1;
        // init stuff before loop
        int travelTime(-1);
        StopId stop(-1);
        Vertex transferStop(-1);
        while (roundBegin < roundEnd && n < 15) {
            // Evaluate final transfers in order to check if the target is reachable
            for (size_t i = roundBegin; i < roundEnd; ++i) {
                const TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; ++j) {
                    stop = data.arrivalEvents[j].stop;
                    // IMPORTANT! continue, not break
                    if (data.arrivalEvents[j].arrivalTime >= minArrivalTimeFastLookUp[stop][n]) continue;
                    addTargetLabel(stop, data.arrivalEvents[j].arrivalTime, i, n, j);
                    AssertMsg(data.raptorData.transferGraph.isVertex(stop), "This stop is not represented in the transfergraph!\n");
                    for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
                        transferStop = data.raptorData.transferGraph.get(ToVertex, edge);
                        if (!data.isStop(transferStop)) continue;
                        travelTime = data.raptorData.transferGraph.get(TravelTime, edge);
                        addTargetLabel(StopId(transferStop), data.arrivalEvents[j].arrivalTime + travelTime, i, n, j);
                    }
                }
            }
            for (size_t i = roundBegin; i < roundEnd; ++i) {
                TripLabel& label = queue[i];
                for (StopEventId j = label.begin; j < label.end; ++j) {
                    if (data.arrivalEvents[j].arrivalTime > minArrivalTimeFastLookUp[data.arrivalEvents[j].stop][n]) continue;
                    for (Edge edge : data.stopEventGraph.edgesFrom(Vertex(j))) {
                        // also pass the fromStopEvent j
                        enqueue(edge, i, n, j);
                    }
                }
            }
            roundBegin = roundEnd;
            roundEnd = queueSize;

            ++n;
        }
    }

    inline void enqueue(const TripId trip, const StopIndex index) noexcept {
        if (reachedIndex.alreadyReached(trip, index, 1)) return;
        const StopEventId firstEvent = data.firstStopEventOfTrip[trip];
        queue[queueSize] = TripLabel(StopEventId(firstEvent + index), StopEventId(firstEvent + reachedIndex(trip, 1)));
        tripLabelEdge[queueSize] = std::make_pair(noEdge, noStopEvent); // noStopEvent since this is the first trip
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(trip, index, 1);
    }

    inline void enqueue(const Edge edge, const size_t parent, const u_int8_t n, const StopEventId fromStopEventId) noexcept {
        const EdgeLabel& label = edgeLabels[edge];
        if (reachedIndex.alreadyReached(label.trip, label.stopEvent - label.firstEvent, n+1)) return;
        queue[queueSize] = TripLabel(label.stopEvent, StopEventId(label.firstEvent + reachedIndex(label.trip, n+1)), parent);
        tripLabelEdge[queueSize] = std::make_pair(edge, fromStopEventId);
        queueSize++;
        AssertMsg(queueSize <= queue.size(), "Queue is overfull!");
        reachedIndex.update(label.trip, StopIndex(label.stopEvent - label.firstEvent), n+1);
    }

    inline void addTargetLabel(const StopId stop, const int newArrivalTime, const u_int32_t parent = -1, const u_int8_t n = 0, const StopEventId fromStopEventId = noStopEvent) noexcept {
        if (newArrivalTime < minArrivalTimeFastLookUp[stop][n]) {
            targetLabels[stop][n].arrivalTime = newArrivalTime;
            targetLabels[stop][n].parent = parent;
            targetLabels[stop][n].fromStopEventId = fromStopEventId;

            targetLabelChanged[stop][n] = true;
            // add stop to update next round
            stopsToUpdate.insert(stop);

            minArrivalTimeFastLookUp[stop][n] = newArrivalTime;

            #pragma omp simd
            for (int i=n+1; i < 16; ++i) {
                if (targetLabels[stop][i].arrivalTime > targetLabels[stop][n].arrivalTime) {
                    targetLabels[stop][i].clear();
                }
                if (minArrivalTimeFastLookUp[stop][i] > newArrivalTime) minArrivalTimeFastLookUp[stop][i] = newArrivalTime;
            }
        }
    }

    inline void emptyBuffer(const StopId target, const int lowerTimeBound, const StopId stop, const bool checkIfTargetLabelChanged = true) noexcept {
        // 1) need to find which targetLabel changed and get the journey description
        // 2) need to "dive" into the journey description for every trip within the time range
        // 3) set every transfer for the target flag
        int targetFlag = data.getPartitionCell(stop);

        alreadyInVector.clear();

        for (u_int8_t n(0); n < targetLabelChanged[target].size(); ++n) {
            if (checkIfTargetLabelChanged && !targetLabelChanged[target][n]) continue;

            std::vector<BufferJourneyLeg>& bestBufferedJourney = bufferedJourneys[stop][n];

            // if there is no journey, or of length 1, skip it
            if (bestBufferedJourney.size() <= 1) continue;

            // loop over trips of first routeId that depart until lowerTimeBound
            // NOTE: bestBufferedJourney is reversed

            std::vector<TripId> tripsToScan = {};
            std::vector<TripId> nextTripsToScan = {};

            // throw all "first" tripIds into the vector
            TripId latestTrip = bestBufferedJourney.back().tripId;
            RouteId currentRouteId = data.routeOfTrip[latestTrip];
            StopIndex stopIndex =  getStopIndexOfStopInRouteId(bestBufferedJourney.back().start, currentRouteId);
            AssertMsg(stopIndex != noStopIndex, "StopIndex has not been found!");
            TripId currentTripId = data.getEarliestTrip(currentRouteId, stopIndex, lowerTimeBound);

            while (currentTripId < latestTrip) {
                tripsToScan.push_back(currentTripId);
                ++currentTripId;
            }

            for (u_int8_t i(0); i < bestBufferedJourney.size() - 1; ++i) {
                BufferJourneyLeg& currentElement = bestBufferedJourney[bestBufferedJourney.size() - 1 - i];
                currentRouteId = data.routeOfTrip[currentElement.tripId];

                // get currentStopIndex of bestBufferedJourney[bestBufferedJourney.size() - 1 - i].end
                stopIndex = getStopIndexOfStopInRouteId(bestBufferedJourney[bestBufferedJourney.size() - 1 - i].end, currentRouteId);
                AssertMsg(stopIndex != noStopIndex, "StopIndex for Stop " << bestBufferedJourney[bestBufferedJourney.size() - 1 - i].end << " and Route " << currentRouteId << " has not been found!\n");
                
                // since we don't need trip that is already flagged
                alreadyInVector[(int) bestBufferedJourney[bestBufferedJourney.size() - 1 - i - 1].tripId] = true;
                RouteId targetRoute = data.routeOfTrip[bestBufferedJourney[bestBufferedJourney.size() - 1 - i - 1].tripId];
                TripId targetTrip(0);

                for (TripId currentTripId : tripsToScan) {
                    // check transfer from currentTripId at stopIndex
                    for (Edge edge : data.stopEventGraph.edgesFrom(Vertex(data.getStopEventId(currentTripId, stopIndex)))) {
                        targetTrip = data.tripOfStopEvent[data.stopEventGraph.get(ToVertex, edge)];
                        if (data.routeOfTrip[targetTrip] == targetRoute) {
                            if (!alreadyInVector[(int) targetTrip]) {
                                nextTripsToScan.push_back(targetTrip);
                                alreadyInVector[(int) targetTrip] = true;
                            }
                            // flag this edge
                            flagsOfThread[edge][targetFlag] = true;
                        }
                    }
                }
                // swap the two vectors
                tripsToScan.clear();
                tripsToScan.swap(nextTripsToScan);
            }
        }
    }

    inline StopIndex getStopIndexOfStopInRouteId(const StopId stop, const RouteId routeId) {
        AssertMsg(data.isStop(stop) && data.isRoute(routeId), "Stop and / or Route are / is not valid!\n");
        StopIndex stopIndex(0);
        for (StopId currentStop : data.raptorData.stopsOfRoute(routeId)) {
            if (currentStop == stop) return stopIndex;
            ++stopIndex;
        }
        Ensure(false, "Stop " << stop << " could not be found in Route " << routeId << "!");
        return noStopIndex;
    }

    inline void getJourneyAndUnwind(const TargetLabel& targetLabel, const int targetCell, const StopId target, const int n, const bool buffer = true) noexcept {
        std::vector<BufferJourneyLeg>& bufferedJourney = bufferedJourneys[target][n];
        bufferedJourney.clear();

        u_int32_t parent = targetLabel.parent;
        if (parent == u_int32_t(-1)) return;

        StopEventId departureStopEvent = noStopEvent;
        Vertex departureStop = target;

        while (parent != u_int32_t(-1)) {
            AssertMsg(parent < queueSize, "Parent " << parent << " is out of range!");
            const TripLabel& label = queue[parent];
            StopEventId arrivalStopEvent;
            Edge edge;
            std::tie(edge, arrivalStopEvent) = (departureStopEvent == noStopEvent) ? std::make_pair(noEdge, targetLabel.fromStopEventId) : getParent(label, StopEventId(departureStopEvent + 1));

            AssertMsg(arrivalStopEvent != noStopEvent, "arrivalStopEvent is null");
            StopId arrivalStop = data.getStopOfStopEvent(arrivalStopEvent);

            departureStopEvent = StopEventId(label.begin - 1);
            departureStop = data.getStopOfStopEvent(departureStopEvent);
            TripId tripId = data.tripOfStopEvent[departureStopEvent];

            if (buffer) bufferedJourney.push_back(BufferJourneyLeg(tripId, StopId(departureStop), arrivalStop));
            parent = label.parent;
            if (edge != noEdge) flagsOfThread[edge][targetCell] = true;
        }
    }

    inline std::pair<Edge, StopEventId> getParent(const TripLabel& parentLabel, const StopEventId departureStopEvent) const noexcept {
        for (StopEventId i = parentLabel.begin; i < parentLabel.end; i++) {
            for (const Edge edge : data.stopEventGraph.edgesFrom(Vertex(i))) {
                if (edgeLabels[edge].stopEvent == departureStopEvent) return std::make_pair(edge, i);
            }
        }
        Ensure(false, "Could not find parent stop event!");
        return std::make_pair(noEdge, noStopEvent);
    }

    /*
    inline void unwindJourney(const TargetLabel& targetLabel, const int targetCell, const int n) noexcept {
        StopEventId departureStopEvent(0);
        StopEventId arrivalStopEvent(0);
        TripId tripId(0);

        u_int32_t parent = targetLabel.parent;

        while (parent != u_int32_t(-1)) {
            AssertMsg(parent < queueSize, "Parent " << parent << " is out of range!");

            const TripLabel& label = queue[parent];
            // this is to get a fast acces of the edge
            std::pair<Edge, StopEventId>& pairOfTripLabel = tripLabelEdge[parent];
            parent = label.parent;

            // add Flags to edge
            if (pairOfTripLabel.first != noEdge) flagsOfThread[pairOfTripLabel.first][targetCell] = true;
        }
    }
    */

private:
    Data& data;

    std::vector<std::vector<bool>>& flagsOfThread;

    int numberOfPartitions;
    std::vector<int> transferFromSource;
    StopId lastSource;

    IndexedSet<false, RouteId> reachedRoutes;

    std::vector<TripLabel> queue;
    size_t queueSize;
    ProfileReachedIndex reachedIndex;

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
    int minDepartureTime;
    int maxDepartureTime;

    std::vector<std::vector<TripStopIndex>>& collectedDepTimes;

    // this is to get the edge and the fromStopEventId  of a triplabel faster
    std::vector<std::pair<Edge, StopEventId>> tripLabelEdge;

    std::vector<TripBased::RouteLabel>& routeLabels;

    // Buffering Journey
    // bufferedJourneys[STOP] = { std::vector<BufferJourneyLeg>, std::vector<BufferJourneyLeg>, (...), std::vector<BufferJourneyLeg>} for all 16 Transfers
    std::vector<std::vector<std::vector<BufferJourneyLeg>>> bufferedJourneys;
    std::vector<std::vector<BufferJourneyLeg>> emptyBufferJourneyForOneStop;

    std::unordered_map<int, bool> alreadyInVector;
};

}
