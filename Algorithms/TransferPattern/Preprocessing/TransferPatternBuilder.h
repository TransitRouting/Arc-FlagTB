#pragma once

#include "../../../DataStructures/Graph/Utils/Utils.h"
#include "../../../DataStructures/TransferPattern/Data.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../TripBased/Query/ProfileOneToAllQuery.h"

#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Vector/Vector.h"

#include <google/dense_hash_map>

/* #include <unordered_map> */

namespace TransferPattern {

    struct eqVecStopid {
        bool operator()(const std::vector<StopId>& a, const std::vector<StopId>& b) const {
            return a == b;
        }
    };

class TransferPatternBuilder {
public:
    TransferPatternBuilder(const TripBased::Data& data)
        : data(data)
        , query(data)
        , dynamicDAG()
        , minDep(0)
        , maxDep(24 * 60 * 60 - 1)
    {
        seenPrefix.set_empty_key({noStop});
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

    // @todo check why so many edges are in the resulting dag? is it because we use TB? and hence many different transfer patterns are created, althought one could reduce it?
    inline void addPrefixToDAG(std::vector<StopId>& prefix, const int travelTime = -1)
    {
        AssertMsg(prefix.size() > 0, "Prefix is empty?");
        if (seenPrefix.find(prefix) != seenPrefix.end())
            return;

        std::vector<StopId> prefixBefore(prefix);
        prefixBefore.pop_back();

        // a bit ugly, but this is to allow A->B via route and A->B via foot
        if (prefixBefore.back() == noStop)
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

            for (size_t i(0); i < j.size(); ++i) {
                leg = j[i];

                if (leg.from == leg.to)
                    continue;

                if (!leg.usesRoute) {
                    travelTime = getTravelTimeByFootpath(StopId(leg.from), StopId(leg.to));
                } else {
                    travelTime = -1;
                }
                if (leg.to == j.back().to) {
                    // add last leg (this is the special stop-vertex)
                    AssertMsg(seenPrefix.find(currentPrefix) != seenPrefix.end(), "Current Prefix is not in the map?");
                    if (!dynamicDAG.hasEdge(Vertex(target), seenPrefix[currentPrefix]))
                        dynamicDAG.addEdge(Vertex(target), seenPrefix[currentPrefix]).set(TravelTime, travelTime);
                    break;
                } else {
                    if (travelTime != -1)
                        currentPrefix.push_back(noStop);
                    currentPrefix.push_back(StopId(leg.to));
                    addPrefixToDAG(currentPrefix, travelTime);
                }
            }
        }

        // keep first # of stops vertices, the rest can vanish
        dynamicDAG.deleteVertices([&](Vertex vertex) { return vertex >= data.raptorData.numberOfStops() && dynamicDAG.isIsolated(vertex); });
        /* dynamicDAG.reduceMultiEdgesBy(TravelTime); */
        dynamicDAG.packEdges();
    }

    inline void clear() noexcept
    {
        dynamicDAG.clear();
        dynamicDAG.reserve(data.numberOfStops() << 3, data.numberOfStops() << 3);
        dynamicDAG.addVertices(data.numberOfStops());
        for (const Vertex vertex : dynamicDAG.vertices()) {
            dynamicDAG.set(ViaVertex, vertex, vertex);
        }

        seenPrefix.resize(1<<6);
        seenPrefix.clear_no_resize();
    }

    inline int getTravelTimeByFootpath(StopId from, StopId to) noexcept
    {
        AssertMsg(data.isStop(from), "From is not a valid stop!");
        AssertMsg(data.isStop(to), "To is not a valid stop!");

        if (from == to)
            return 0;

        Edge usedEdge = data.raptorData.transferGraph.findEdge(Vertex(from), Vertex(to));
        AssertMsg(data.raptorData.transferGraph.isEdge(usedEdge), "Transfer not valid? From " << from << " to " << to << "\n");
        return data.raptorData.transferGraph.get(TravelTime, usedEdge);
    }

private:
    const TripBased::Data data;
    TripBased::ProfileOneToAllQuery<TripBased::NoProfiler> query;

    DynamicDAGTransferPattern dynamicDAG;

    /* std::unordered_map<std::vector<StopId>, Vertex, std::VectorHasher<StopId>> seenPrefix; */
    google::dense_hash_map<std::vector<StopId>, Vertex, std::VectorHasher<StopId>, eqVecStopid> seenPrefix;
    const int minDep;
    const int maxDep;
};

inline void ComputeTransferPatternUsingTripBased(TripBased::Data& data, TransferPattern::Data& tpData)
{
    Progress progress(data.numberOfStops());
    TransferPatternBuilder bobTheBuilder(data);

    for (const StopId stop : data.stops()) {
        bobTheBuilder.computeTransferPatternForStop(stop);
        AssertMsg(Graph::isAcyclic<DynamicDAGTransferPattern>(bobTheBuilder.getDAG()), "Graph is not acyclic!");

        Graph::move(std::move(bobTheBuilder.getDAG()), tpData.transferPatternOfStop[stop]);
        tpData.transferPatternOfStop[stop].sortEdges(ToVertex);

        ++progress;
    }
    progress.finished();
}

inline void ComputeTransferPatternUsingTripBased(TripBased::Data& data, TransferPattern::Data& tpData, const int numberOfThreads, const int pinMultiplier = 1)
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
            bobTheBuilder.computeTransferPatternForStop(StopId(i));
            AssertMsg(Graph::isAcyclic<DynamicDAGTransferPattern>(bobTheBuilder.getDAG()), "Graph is not acyclic!");

            Graph::move(std::move(bobTheBuilder.getDAG()), tpData.transferPatternOfStop[i]);
            tpData.transferPatternOfStop[i].sortEdges(ToVertex);

            ++progress;
        }
    }
    progress.finished();
}
}
