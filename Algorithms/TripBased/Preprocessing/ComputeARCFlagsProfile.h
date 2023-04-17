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

#include <numeric>

#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../DataStructures/RAPTOR/Entities/StopEvent.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/String/String.h"
#include "CalculateARCFlagsProfile.h"

namespace TripBased {

class ComputeARCFlagsProfile {
public:
    ComputeARCFlagsProfile(Data& data, const int numberOfThreads, const int pinMultiplier = 1)
        : data(data)
        , numberOfThreads(numberOfThreads)
        , pinMultiplier(pinMultiplier)
        , routeLabels(data.numberOfRoutes())
    {
        collectedDepTimes.assign(data.numberOfStops(), {});
        Graph::copy(data.stopEventGraph, stopEventGraphDynamic);

        // copy flags for every thread => bitWiseOr in the end
        std::vector<bool> emptyFlagOneEdge(data.getNumberOfPartitionCells(), false);
        std::vector<std::vector<bool>> initialFlags(data.stopEventGraph.numEdges(), emptyFlagOneEdge);
        flagsPerThread.assign(numberOfThreads, initialFlags);

        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const TripId firstTrip = data.firstTripOfRoute[route];
            const RAPTOR::StopEvent* stopEvents = data.eventArrayOfTrip(firstTrip);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; trip++) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; stopIndex++) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] = stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }
    }

    void computeARCFlags(const bool fixingDepTime = true, const bool buffer = true, const bool verbose = true)
    {
        if (verbose) {
            std::cout << "Computing ARCFlags with " << numberOfThreads << " threads." << std::endl;
        }
        int maxDepartureTime = 24 * 60 * 60 - 1;
        int minDepartureTime = 0;
        collectAllDepTimes(minDepartureTime, maxDepartureTime, true);

        if (verbose)
            std::cout << "Starting the computation!\n";
        Progress progress(data.numberOfStops());

        const int numCores = numberOfCores();
        omp_set_num_threads(numberOfThreads);
#pragma omp parallel
        {
            int threadId = omp_get_thread_num();
            pinThreadToCoreId((threadId * pinMultiplier) % numCores);
            AssertMsg(omp_get_num_threads() == numberOfThreads,
                "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

            CalculateARCFlagsProfile calcARCFlag(data, flagsPerThread[threadId], collectedDepTimes, minDepartureTime,
                maxDepartureTime, routeLabels);

// one thread handles one stop
#pragma omp for schedule(dynamic)
            for (size_t stop = 0; stop < data.numberOfStops(); ++stop) {
                calcARCFlag.run(Vertex(stop), buffer);
                ++progress;
            }
        }

        progress.finished();

        // now bitwiseor every edge
        std::vector<std::vector<bool>>& toSetFlags(stopEventGraphDynamic.get(ARCFlag));

        // idea: one thread handles one edge => goes through all flags of threads
        omp_set_num_threads(numberOfThreads);
#pragma omp parallel
        {
            int threadId = omp_get_thread_num();
            pinThreadToCoreId((threadId * pinMultiplier) % numCores);
            AssertMsg(omp_get_num_threads() == numberOfThreads,
                "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

#pragma omp for schedule(dynamic)
            for (size_t edge = 0; edge < data.stopEventGraph.numEdges(); ++edge) {
                for (int k = 0; k < data.getNumberOfPartitionCells(); ++k) {
                    int threadFlagId(0);
                    bool isSet = false;
                    while (!isSet && threadFlagId < numberOfThreads) {
                        AssertMsg((size_t)threadFlagId < flagsPerThread.size(), "threadFlagId out of bounds!\n");
                        AssertMsg((size_t)edge < flagsPerThread[threadFlagId].size(), "edge out of bounds!\n");
                        AssertMsg((size_t)k < flagsPerThread[threadFlagId][edge].size(), "k out of bounds!\n");
                        isSet = isSet | flagsPerThread[threadFlagId++][edge][k];
                    }
                    AssertMsg(edge < toSetFlags.size(), "edge out of bounds!\n");
                    AssertMsg((size_t)k < toSetFlags[edge].size(), "edge out of bounds!\n");
                    toSetFlags[edge][k] = isSet;
                }
            }
        }
        // free up memory
        std::vector<std::vector<std::vector<bool>>>().swap(flagsPerThread);

        if (fixingDepTime) {
            if (verbose)
                std::cout << "Done! Now fixing the 'departure profile problem'!\n";
            Progress fixingDepProgress(data.numberOfRoutes());

            omp_set_num_threads(numberOfThreads);
#pragma omp parallel
            {
                int threadId = omp_get_thread_num();
                pinThreadToCoreId((threadId * pinMultiplier) % numCores);
                AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

#pragma omp for schedule(dynamic)
                for (size_t routeIndex = 0; routeIndex < data.numberOfRoutes(); ++routeIndex) {
                    const RouteId route = RouteId(routeIndex);
                    Range<TripId> tripsOfRoute = data.tripsOfRoute(route);
                    if (tripsOfRoute.size() == 1)
                        continue;
                    size_t numberOfStopsInRoute = data.numberOfStopsInRoute(route);
                    int tripIndex(tripsOfRoute.size() - 1);

                    while (tripIndex > 0) {
                        for (StopIndex stop(0); stop < numberOfStopsInRoute; ++stop) {
                            const StopEventId currentStopEventId = data.getStopEventId(tripsOfRoute[tripIndex], stop);
                            Range<Edge> edgesToInsert = stopEventGraphDynamic.edgesFrom(Vertex(currentStopEventId));
                            for (Edge edge : edgesToInsert) {
                                if (numberOfFLAGInEdge(stopEventGraphDynamic.get(ARCFlag, edge)) == 0)
                                    continue;
                                for (int earlierTripIndex(tripIndex - 1); earlierTripIndex >= 0; --earlierTripIndex) {
                                    Vertex fromVertex = Vertex(data.getStopEventId(tripsOfRoute[earlierTripIndex], stop));
                                    Edge originalEdge = stopEventGraphDynamic.findEdge(
                                        fromVertex, Vertex(stopEventGraphDynamic.get(ToVertex, edge)));
                                    if (originalEdge != noEdge) {
                                        bitWiseOr(toSetFlags[originalEdge], toSetFlags[edge]);
                                    }
                                }
                            }
                        }
                        --tripIndex;
                    }
                    ++fixingDepProgress;
                }
            }

            fixingDepProgress.finished();
        }

        if (verbose)
            std::cout << "Preprocessing done!\nNow deleting unnecessary edges\n";
        size_t flagCounter(0);
        size_t deletedEdges(0);
        std::vector<bool> edgesToDelete(stopEventGraphDynamic.edgeLimit(), false);

        for (Edge edge : stopEventGraphDynamic.edges()) {
            // if no flag set for a particular edge => throw it away
            if (numberOfFLAGInEdge(stopEventGraphDynamic.get(ARCFlag, edge)) == 0) {
                edgesToDelete[edge] = true;
                ++deletedEdges;
            } else
                flagCounter += numberOfFLAGInEdge(stopEventGraphDynamic.get(ARCFlag, edge));
        }

        stopEventGraphDynamic.deleteEdges(edgesToDelete, true);
        Graph::copy(stopEventGraphDynamic, data.stopEventGraph);

        if (verbose) {
            std::cout << "ARC-FLAG Stats:\n";
            std::cout << "Number of Flags set:          " << flagCounter << " ("
                      << 100 * flagCounter / (stopEventGraphDynamic.numEdges() * data.getNumberOfPartitionCells())
                      << "%)\n";
            std::cout << "Number of removed edges:      " << deletedEdges << " ("
                      << 100 * deletedEdges / (stopEventGraphDynamic.numEdges() + deletedEdges) << "%)\n";
        }
    }

    int numberOfFLAGInEdge(std::vector<bool>& flags)
    {
        return std::accumulate(flags.begin(), flags.end(), 0);
    }

    const std::vector<TripStopIndex>& getCollectedDepTimes(const StopId& stop)
    {
        return collectedDepTimes[stop];
    }

    void bitWiseOr(std::vector<bool>& to, const std::vector<bool>& from)
    {
        AssertMsg(to.size() == from.size(), "BitWiseOr - the two vectors have different length!\n");
#pragma omp simd
        for (size_t i = 0; i < to.size(); ++i) {
            to[i] = to[i] | from[i];
        }
    }

    void collectAllDepTimes(const int minDepartureTime = 0, const int maxDepartureTime = 86399,
        const bool verbose = true) noexcept
    {
        if (verbose)
            std::cout << "Start by collecting all the departure stopevents into the approriate stop bucket!\n";
        for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
            collectedDepTimes[stop].clear();
            collectedDepTimes[stop].reserve((int)data.numberOfTrips()); // adjustable
        }

        Progress progress(data.numberOfRoutes());
        for (const RouteId route : data.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const StopId* stops = data.raptorData.stopArrayOfRoute(route);
            const TripId firstTrip = data.firstTripOfRoute[route];
            const RAPTOR::StopEvent* stopEvents = data.eventArrayOfTrip(firstTrip);
            for (size_t stopEventIndex = 0; stopEventIndex < data.raptorData.numberOfStopEventsInRoute(route);
                 ++stopEventIndex) {
                if ((stopEventIndex + 1) % numberOfStops == 0)
                    continue;
                const StopId stop = stops[stopEventIndex % numberOfStops];
                // add it to the current stop
                int departureTime = stopEvents[stopEventIndex].departureTime;
                // seems a little weird - it could be that a departure is outside
                // the maxDepartureTime, but due to travel is is legal
                if (!(departureTime < minDepartureTime || departureTime >= maxDepartureTime)) {
                    collectedDepTimes[stop].push_back(
                        TripStopIndex(TripId(firstTrip + (int)(stopEventIndex / numberOfStops)),
                            StopIndex(stopEventIndex % numberOfStops), departureTime));
                }
                // now for every transfer reachable stop
                for (const Edge edge : data.raptorData.transferGraph.edgesFrom(stop)) {
                    const Vertex transferStop = data.raptorData.transferGraph.get(ToVertex, edge);
                    const int walkingTime = data.raptorData.transferGraph.get(TravelTime, edge);
                    AssertMsg(walkingTime != INFTY, "Walking Time is infinity, which does not make sense!\n");
                    int transferDepTime = departureTime - walkingTime;
                    if (transferDepTime < minDepartureTime || transferDepTime >= maxDepartureTime)
                        continue;
                    collectedDepTimes[transferStop].push_back(
                        TripStopIndex(TripId(firstTrip + (int)(stopEventIndex / numberOfStops)),
                            StopIndex(stopEventIndex % numberOfStops), transferDepTime));
                }
            }
            ++progress;
        }

        for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
            std::stable_sort(collectedDepTimes[stop].begin(), collectedDepTimes[stop].end(),
                [](const TripStopIndex a, const TripStopIndex b) { return a.depTime > b.depTime; });
        }

        progress.finished();
    }

private:
    std::vector<std::vector<std::vector<bool>>> flagsPerThread;
    std::vector<std::vector<size_t>> counterPerThread;
    Data& data;
    const int numberOfThreads;
    const int pinMultiplier;
    std::vector<TripBased::RouteLabel> routeLabels;
    DynamicTransferGraphWithARCFlag stopEventGraphDynamic;
    std::vector<std::vector<TripStopIndex>> collectedDepTimes;
};
} // namespace TripBased
