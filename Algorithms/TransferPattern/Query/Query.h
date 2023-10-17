#pragma once

#include "../../../DataStructures/TransferPattern/Data.h"
#include "Profiler.h"

#include <vector>

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
        , alreadySeen(data.maxNumberOfVerticesInTP())
    {
        profiler.registerPhases({ PHASE_EXTRACT_QUERY_GRAPH, PHASE_CLEAR, PHASE_EVAL_GRAPH });
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

        /* evaluateQueryGraph(); */
        profiler.done();
    }

    inline void clear() noexcept
    {
        profiler.startPhase();

        queryGraph.clear();
        // should prop be evaluated which reserve size is fastest
        queryGraph.reserve(1 << 8, 1 << 8);

        left = 0;
        right = 0;

        alreadySeen.assign(data.maxNumberOfVerticesInTP(), noVertex);

        profiler.donePhase(PHASE_CLEAR);
    }

    inline void extractQueryGraph()
    {
        profiler.startPhase();

        const StaticDAGTransferPattern& sourceTP = data.transferPatternOfStop[sourceStop];

        Vertex currentVertex(targetStop);

        addVertexToQueryGraph(currentVertex, sourceTP);

        while (queueIsNotEmpty()) {
            currentVertex = popFront();

            // relax all outgoing edges and add (reversed edge) to queryGraph
            for (const Edge edge : sourceTP.edgesFrom(currentVertex)) {
                Vertex successor = sourceTP.get(ToVertex, edge);

                // insert into queue
                if (alreadySeen[successor] == noVertex) [[likely]]
                    addVertexToQueryGraph(successor, sourceTP);

                // add edges
                queryGraph.addEdge(alreadySeen[successor], alreadySeen[currentVertex]).set(TravelTime, sourceTP.get(TravelTime, edge));
                profiler.countMetric(METRIC_NUM_EDGES_QUERY_GRAPH);
            }
        }

        profiler.donePhase(PHASE_EXTRACT_QUERY_GRAPH);
    }

    inline void addVertexToQueryGraph(Vertex vertex, const StaticDAGTransferPattern& sourceTP)
    {
        insertIntoQueue(vertex);

        queryGraph.addVertex();
        alreadySeen[vertex] = Vertex(queryGraph.numVertices() - 1);

        AssertMsg(queryGraph.isVertex(alreadySeen[vertex]), "Vertex is not really a vertex!");

        queryGraph.set(ViaVertex, alreadySeen[vertex], sourceTP.get(ViaVertex, vertex));

        profiler.countMetric(METRIC_NUM_VERTICES_QUERY_GRAPH);
    }

    // queue ops
    inline void insertIntoQueue(Vertex vertex)
    {
        AssertMsg(right <= queue.size(), "Right is out of bounds!");
        if (right == queue.size()) {
            // the queue is full, so push back resizes
            ++right;
            queue.push_back(vertex);
        } else {
            queue[right++] = vertex;
        }
    }

    inline Vertex popFront() noexcept
    {
        AssertMsg(left < queue.size(), "Left is out of bounds!");
        return queue[left++];
    }

    inline bool queueIsNotEmpty() noexcept
    {
        return left < right;
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
    std::vector<Vertex> alreadySeen;

    Profiler profiler;
};

}
