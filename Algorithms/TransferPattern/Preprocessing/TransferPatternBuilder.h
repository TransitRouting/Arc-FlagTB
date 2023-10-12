#pragma once

#include "../../../DataStructures/TripBased/Data.h"
#include "../../TripBased/Query/ProfileOneToAllQuery.h"

#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"

namespace TransferPattern {

class TransferPatternBuilder {
public:
    TransferPatternBuilder(const TripBased::Data& data)
        : data(data)
        , query(data)
        , minDep(0)
        , maxDep(24 * 60 * 60 - 1)
    {
        dynamicDAG.addVertices(data.raptorData.numberOfStops());
    }

    inline const DynamicDAGTransferPattern& getDAG() const noexcept
    {
        return dynamicDAG;
    }

    inline DynamicDAGTransferPattern& getDAG() noexcept
    {
        return dynamicDAG;
    }

    inline void incorperateJourney(RAPTOR::Journey& journey)
    {
        StopId start(journey[0].from);
        StopId target(journey.back().to);

        for (RAPTOR::JourneyLeg& leg : journey) {
            if (leg.from == leg.to)
                continue;

            if (leg.usesRoute) {
                // TODO check how to add 'circle' vertices
                dynamicDAG.addEdge(Vertex(leg.from), Vertex(leg.to)).set(TravelTime, -1);
            } else {
                Edge usedEdge = data.raptorData.transferGraph.findEdge(Vertex(leg.from), Vertex(leg.to));
                AssertMsg(data.raptorData.transferGraph.isEdge(usedEdge), "Transfer not valid?");
                dynamicDAG.addEdge(Vertex(leg.from), Vertex(leg.to)).set(TravelTime, data.raptorData.transferGraph.get(TravelTime, usedEdge));
            }
        }
    }

    inline void computeTransferPatternForStop(const StopId stop = noStop)
    {
        AssertMsg(data.raptorData.isStop(stop), "Stop is not valid!");

        std::vector<StopId> targets;
        targets.reserve(data.numberOfStops());

        for (const StopId target : data.stops()) {
            if (target == stop)
                continue;
            targets.push_back(target);
        }

        query.run(stop, minDep, maxDep, targets);

        std::vector<RAPTOR::Journey> journeys = query.getAllJourneys();

        for (RAPTOR::Journey& j : journeys) {
            incorperateJourney(j);
        }
    }

private:
    const TripBased::Data data;
    TripBased::ProfileOneToAllQuery<TripBased::NoProfiler> query;

    const int minDep;
    const int maxDep;

    DynamicDAGTransferPattern dynamicDAG;
};

inline void ComputeTransferPatternUsingTripBased(TripBased::Data& data) noexcept
{
    Progress progress(data.numberOfStops());
    TransferPatternBuilder bobTheBuilder(data);

    for (const StopId stop : data.stops()) {
        bobTheBuilder.computeTransferPatternForStop(stop);
        ++progress;
    }
    /* Graph::move(std::move(builder.getStopEventGraph()), data.stopEventGraph); */
    /* data.stopEventGraph.sortEdges(ToVertex); */
    progress.finished();
}

inline void ComputeTransferPatternUsingTripBased(TripBased::Data& data, const int numberOfThreads, const int pinMultiplier = 1) noexcept
{
    Progress progress(data.numberOfStops());

    /* SimpleEdgeListWithARCFlag stopEventGraph; */
    /* stopEventGraph.addVertices(data.numberOfStopEvents()); */

    const int numCores = numberOfCores();

    omp_set_num_threads(numberOfThreads);
#pragma omp parallel
    {
        int threadId = omp_get_thread_num();
        pinThreadToCoreId((threadId * pinMultiplier) % numCores);
        AssertMsg(omp_get_num_threads() == numberOfThreads,
            "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

        TransferPatternBuilder bobTheBuilder(data);
        const size_t numberOfStops = data.numberOfStops();

#pragma omp for schedule(dynamic, 1)
        for (size_t i = 0; i < numberOfStops; ++i) {
            const StopId stop = StopId(i);
            bobTheBuilder.computeTransferPatternForStop(stop);
            ++progress;
        }

        /* #pragma omp critical */
        /*         { */
        /*             for (const auto [edge, from] : builder.getStopEventGraph().edgesWithFromVertex()) { */
        /*                 // Arc-Flag TB set empty flags */
        /*                 std::vector<bool> emptyFlags(data.getNumberOfPartitionCells(), false); */
        /*                 stopEventGraph.addEdge(from, builder.getStopEventGraph().get(ToVertex, edge)).set(ARCFlag, emptyFlags); */
        /*             } */
        /*         } */
        /*     } */

        /* Graph::move(std::move(stopEventGraph), data.stopEventGraph); */
        /* data.stopEventGraph.sortEdges(ToVertex); */
        progress.finished();
    }
}
}
