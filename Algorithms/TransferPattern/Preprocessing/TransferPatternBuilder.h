#pragma once

#include "../../../DataStructures/TransferPattern/Data.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../TripBased/Query/ProfileOneToAllQuery.h"

#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/Vector/Vector.h"
#include "../../../Helpers/MultiThreading.h"

#include <unordered_map>

namespace TransferPattern {

class TransferPatternBuilder {
public:
    TransferPatternBuilder(const TripBased::Data& data)
        : data(data)
        , query(data)
        , dynamicDAG()
        , seenPrefix()
        , minDep(0)
        , maxDep(24 * 60 * 60 - 1)
    {
        clear();
    }

    inline const DynamicDAGTransferPattern& getDAG() const noexcept
    {
        return dynamicDAG;
    }

    inline DynamicDAGTransferPattern& getDAG() noexcept
    {
        return dynamicDAG;
    }

    inline void addPrefixToDAG(std::vector<StopId>& prefix, const int travelTime = -1) {
        AssertMsg(prefix.size() > 0, "Prefix is empty?");
        if (seenPrefix.find(prefix) != seenPrefix.end())
            return;

        std::vector<StopId> prefixBefore(prefix);
        prefixBefore.pop_back();

        Vertex beforeVertex = seenPrefix[prefixBefore];
        Vertex newVertex = dynamicDAG.addVertex();
        dynamicDAG.set(ViaVertex, newVertex, Vertex(prefix.back()));

        seenPrefix[prefix] = newVertex;

        dynamicDAG.addEdge(newVertex, beforeVertex).set(TravelTime, travelTime);
    }

    inline void computeTransferPatternForStop(const StopId stop = noStop)
    {
        AssertMsg(data.raptorData.isStop(stop), "Stop is not valid!");
        clear();


        std::vector<StopId> currentPrefix;
        currentPrefix.reserve(32);
        currentPrefix.push_back(stop);
        seenPrefix[currentPrefix] = Vertex(stop);

        // This solves one-to-all
        query.run(Vertex(stop), minDep, maxDep);

        int travelTime(-1);
        StopId target(0);
        RAPTOR::JourneyLeg leg;

        for (RAPTOR::Journey& j : query.getAllJourneys()) {
            currentPrefix.clear();
            currentPrefix.reserve(32);

            currentPrefix.push_back(stop);
            target = StopId(j.back().to);

            for (size_t i(0); i < j.size() - 1; ++i) {
                leg = j[i];
                if (leg.from == leg.to)
                    continue;

                // TODO check if it is possible if a prefix ABCD has B->C using a route and another prefix ABCD B->C as footpath
                currentPrefix.push_back(StopId(leg.to));

                if (!leg.usesRoute) {
                    travelTime = getTravelTimeByFootpath(StopId(leg.from), StopId(leg.to));
                } else {
                    travelTime = -1;
                }
                addPrefixToDAG(currentPrefix, travelTime);
            }

            // add last leg (this is the special stop-vertex)
            AssertMsg(seenPrefix.find(currentPrefix) != seenPrefix.end(), "Current Prefix is not in the map?");

            int travelTime(-1);
            if (!j.back().usesRoute) {
                travelTime = getTravelTimeByFootpath(currentPrefix.back(), target);
            }
            dynamicDAG.addEdge(Vertex(target), seenPrefix[currentPrefix]).set(TravelTime, travelTime);
        }

        dynamicDAG.deleteIsolatedVertices();
    }

    inline void clear() noexcept
    {
        dynamicDAG.clear();
        dynamicDAG.addVertices(data.numberOfStops());
        for (const Vertex vertex : dynamicDAG.vertices()) {
            dynamicDAG.set(ViaVertex, vertex, vertex);
        }

        seenPrefix.clear();
    }

    inline int getTravelTimeByFootpath(StopId from, StopId to) noexcept {
        AssertMsg(data.isStop(from), "From is not a valid stop!");
        AssertMsg(data.isStop(to), "To is not a valid stop!");
        
        if (from == to)
            return 0;


        Edge usedEdge = data.raptorData.transferGraph.findEdge(Vertex(from), Vertex(to));
        AssertMsg(data.raptorData.transferGraph.isEdge(usedEdge), "Transfer not valid?");
        return data.raptorData.transferGraph.get(TravelTime, usedEdge);
    }

private:
    const TripBased::Data data;
    TripBased::ProfileOneToAllQuery<TripBased::NoProfiler> query;

    DynamicDAGTransferPattern dynamicDAG;

    std::unordered_map<std::vector<StopId>, Vertex, std::VectorHasher<StopId>> seenPrefix;
    const int minDep;
    const int maxDep;

};

inline void ComputeTransferPatternUsingTripBased(TripBased::Data& data, TransferPattern::Data& tpData) noexcept
{
    Progress progress(data.numberOfStops());
    TransferPatternBuilder bobTheBuilder(data);

    for (const StopId stop : data.stops()) {
        bobTheBuilder.computeTransferPatternForStop(stop);
        Graph::move(std::move(bobTheBuilder.getDAG()), tpData.transferPatternOfStop[stop]);
        tpData.transferPatternOfStop[stop].sortEdges(ToVertex);
        ++progress;
    }
    progress.finished();
}

inline void ComputeTransferPatternUsingTripBased(TripBased::Data& data, TransferPattern::Data& tpData, const int numberOfThreads, const int pinMultiplier = 1) noexcept
{
    Progress progress(data.numberOfStops());

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

            Graph::move(std::move(bobTheBuilder.getDAG()), tpData.transferPatternOfStop[stop]);
            tpData.transferPatternOfStop[stop].sortEdges(ToVertex);
            ++progress;
        }
    }
    progress.finished();
}
}
