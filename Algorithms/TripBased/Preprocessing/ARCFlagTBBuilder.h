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

class ARCFlagTBBuilder {
public:
    ARCFlagTBBuilder(Data& data, const int numberOfThreads, const int pinMultiplier = 1)
        : data(data)
        , numberOfThreads(numberOfThreads)
        , pinMultiplier(pinMultiplier)
        , routeLabels(data.numberOfRoutes())
    {
        collectedDepTimes.assign(data.numberOfStops(), {});
        Graph::copy(data.stopEventGraph, stopEventGraphDynamic);

        std::vector<uint8_t> emptyFlagOneEdge(data.getNumberOfPartitionCells(), 0);
        uint8InitialFlags.assign(data.stopEventGraph.numEdges(), emptyFlagOneEdge);

        for (const RouteId route : data.raptorData.routes()) {
            const size_t numberOfStops = data.numberOfStopsInRoute(route);
            const size_t numberOfTrips = data.raptorData.numberOfTripsInRoute(route);
            const TripId firstTrip = data.firstTripOfRoute[route];
            const RAPTOR::StopEvent* stopEvents = data.eventArrayOfTrip(firstTrip);
            routeLabels[route].numberOfTrips = numberOfTrips;
            routeLabels[route].departureTimes.resize((numberOfStops - 1) * numberOfTrips);
            for (size_t trip = 0; trip < numberOfTrips; ++trip) {
                for (size_t stopIndex = 0; stopIndex + 1 < numberOfStops; ++stopIndex) {
                    routeLabels[route].departureTimes[(stopIndex * numberOfTrips) + trip] = stopEvents[(trip * numberOfStops) + stopIndex].departureTime;
                }
            }
        }
    }

    void computeARCFlags(const bool verbose = true)
    {
        Assert(data.getNumberOfPartitionCells() > 1);
        if (verbose) {
            std::cout << "Computing ARCFlags with " << numberOfThreads << " threads." << std::endl;
        }
        int maxDepartureTime = 24 * 60 * 60;
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
            AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

            CalculateARCFlagsProfile bobTheBuilder(data, uint8InitialFlags, collectedDepTimes,
                routeLabels);

#pragma omp for schedule(dynamic)
            for (size_t stop = 0; stop < data.numberOfStops(); ++stop) {
                bobTheBuilder.run(Vertex(stop));
                ++progress;
            }
        }

        progress.finished();

        // std::vector<std::vector<bool>>& toSetFlags(stopEventGraphDynamic.get(ARCFlag));
        for (Edge edge : stopEventGraphDynamic.edges()) {
            std::vector<bool> flags(data.getNumberOfPartitionCells(), false);

            for (int i(0); i < data.getNumberOfPartitionCells(); ++i) {
                flags[i] = (bool)uint8InitialFlags[edge][i];
            }
            stopEventGraphDynamic.set(ARCFlag, edge, flags);
            /* for (int i(0); i < data.getNumberOfPartitionCells(); ++i) {
                    toSetFlags[edge][i] = (bool)uint8InitialFlags[edge][i];
            } */
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
            std::cout << "Arc-Flag Stats:\n";
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

    void collectAllDepTimes(const int minDepartureTime = 0, const int maxDepartureTime = 86400,
        const bool verbose = true) noexcept
    {
        if (verbose)
            std::cout << "Start by collecting all the departure stopevents into the approriate stop bucket!\n";
        for (StopId stop(0); stop < data.numberOfStops(); ++stop) {
            collectedDepTimes[stop].clear();
            // collectedDepTimes[stop].reserve((int)data.numberOfTrips() >> 3); // adjustable
            collectedDepTimes[stop].reserve(1000); // adjustable
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
    std::vector<std::vector<uint8_t>> uint8InitialFlags;
    Data& data;
    const int numberOfThreads;
    const int pinMultiplier;
    std::vector<TripBased::RouteLabel> routeLabels;
    DynamicTransferGraphWithARCFlag stopEventGraphDynamic;
    std::vector<std::vector<TripStopIndex>> collectedDepTimes;
};
} // namespace TripBased
