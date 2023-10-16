#pragma once

#include "../../../DataStructures/TransferPattern/Data.h"
#include "Profiler.h"

#include <unordered_map>

namespace TransferPattern {

template <typename PROFILER = NoProfiler>
class Query {
public:
    using Profiler = PROFILER;
    using Type = Query<Profiler>;

public:
    Query(const Data& data)
        : data(data)
        , sourceStop(noVertex)
        , targetStop(noVertex)
        , sourceDepartureTime(0)
        , queryGraph()
        , queue(data.raptorData.numberOfStops()) // what size for the queue?
        , left(0)
        , right(0)
        , alreadySeen()
    {
        profiler.registerPhases({ PHASE_EXTRACT_QUERY_GRAPH, PHASE_EVAL_GRAPH });
        profiler.registerMetrics({ METRIC_NUM_VERTICES_QUERY_GRAPH, METRIC_NUM_EDGES_QUERY_GRAPH, METRIC_RELAXED_EDGES });
    }

    inline void run(const Vertex source, const int departureTime, const Vertex target)
    {
        AssertMsg(data.raptorData.isStop(source), "Source " << (int)source << " is not a valid stop!");
        AssertMsg(data.raptorData.isStop(target), "Target " << (int)target << " is not a valid stop!");

        run(StopId(source), departureTime, StopId(target));
    }

    inline void run(StopId source, int departureTime, StopId target)
    {
        profiler.start();

        clear();
        sourceStop = source;
        targetStop = target;
        sourceDepartureTime = departureTime;

        extractQueryGraph();
        profiler.done();
    }

    inline void clear() noexcept
    {
        queryGraph.clear();
        // should prop be evaluated which reserve size is fastest
        queryGraph.reserve(data.raptorData.numberOfStops(), data.raptorData.numberOfStops());

        left = 0;
        right = 0;

        alreadySeen.clear();
    }

    inline void extractQueryGraph() noexcept
    {
        profiler.startPhase();

        const StaticDAGTransferPattern& sourceTP = data.transferPatternOfStop[sourceStop];

        Vertex currentVertex(targetStop);
        Vertex vertexInQueryGraph(noVertex);

        insertIntoQueue(currentVertex);

        queryGraph.addVertex();
        alreadySeen[(int)currentVertex] = Vertex(queryGraph.numVertices() - 1);

        AssertMsg(queryGraph.isVertex(alreadySeen[(int)currentVertex]), "Vertex is not really a vertex!");

        queryGraph.set(ViaVertex, alreadySeen[(int)currentVertex], sourceTP.get(ViaVertex, currentVertex));
        profiler.countMetric(METRIC_NUM_VERTICES_QUERY_GRAPH);

        while (!queueIsEmpty()) {
            currentVertex = popFront();

            // relax all outgoing edges and add (reversed edge) to queryGraph
            for (const Edge edge : sourceTP.edgesFrom(currentVertex)) {
                Vertex successor = sourceTP.get(ToVertex, edge);
                // insert into queue
                if (alreadySeen.find((int)successor) == alreadySeen.end()) [[likely]] {
                    insertIntoQueue(successor);

                    queryGraph.addVertex();
                    alreadySeen[(int)successor] = Vertex(queryGraph.numVertices() - 1);

                    AssertMsg(queryGraph.isVertex(alreadySeen[(int)currentVertex]), "Vertex is not really a vertex!");

                    queryGraph.set(ViaVertex, alreadySeen[(int)successor], sourceTP.get(ViaVertex, successor));

                    profiler.countMetric(METRIC_NUM_VERTICES_QUERY_GRAPH);
                }

                // add edges
                queryGraph.addEdge(alreadySeen[(int)successor], alreadySeen[(int)currentVertex]).set(TravelTime, sourceTP.get(TravelTime, edge));
                profiler.countMetric(METRIC_NUM_EDGES_QUERY_GRAPH);
            }
        }

        profiler.donePhase(PHASE_EXTRACT_QUERY_GRAPH);
    }

    // queue operations
    inline void insertIntoQueue(Vertex vertex)
    {
        if (right == queue.size()) {
            // the queue is full, so push back resizes
            ++right;
            queue.push_back(vertex);
        } else {
            AssertMsg(right < queue.size(), "Right is out of bounds!");
            queue[right++] = vertex;
        }
    }

    inline Vertex accesFront()
    {
        AssertMsg(left < queue.size(), "Left is out of bounds!");
        return queue[left];
    }

    inline Vertex popFront() noexcept
    {
        AssertMsg(left < queue.size(), "Left is out of bounds!");
        return queue[left++];
    }

    inline bool queueIsEmpty() noexcept
    {
        return left == right;
    }

    inline Profiler& getProfiler() noexcept
    {
        return profiler;
    }

private:
    const Data& data;

    Vertex sourceStop;
    Vertex targetStop;
    int sourceDepartureTime;

    DynamicDAGTransferPattern queryGraph;
    std::vector<Vertex> queue;
    size_t left, right;
    std::unordered_map<int, Vertex> alreadySeen;

    Profiler profiler;
};

}
