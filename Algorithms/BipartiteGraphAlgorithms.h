#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "MaxFlowMinCut/Dinic.h"

#include "../DataStructures/Container/Set.h"
#include "../DataStructures/Graph/Graph.h"
#include "../DataStructures/MaxFlowMinCut/FlowGraphs.h"

#include "../Helpers/Assert.h"
#include "../Helpers/Types.h"
#include "../Helpers/Vector/Permutation.h"
#include "../Helpers/Vector/Vector.h"

class BipartiteGraph {

public:
    BipartiteGraph() { }

    BipartiteGraph(DynamicFlowGraph& graph, IndexedSet<false, Vertex>& leftSide, IndexedSet<false, Vertex>& rightSide)
        : graph(graph)
        , leftSide(leftSide)
        , rightSide(rightSide)
    {
        AssertMsg(checkSideContainsOnlyVerticesInGraph(leftSide), "LeftSide contains vertices not present in the graph!");
        AssertMsg(checkSideContainsOnlyVerticesInGraph(rightSide), "RightSide contains vertices not present in the graph!");
    }

    template <typename GRAPH>
    BipartiteGraph(const GRAPH& originalGraph)
        : graph(Graph::generateFlowGraph(originalGraph, true))
        , leftSide(originalGraph.numVertices())
        , rightSide(originalGraph.numVertices())
    {
        std::vector<Vertex> unassignedVertices = Vector::id<Vertex>(graph.numVertices());
        while (!unassignedVertices.empty()) {
            const Vertex from = unassignedVertices.back();
            unassignedVertices.pop_back();
            if (!leftSide.contains(from))
                rightSide.insert(from);
            IndexedSet<false, Vertex>& otherSide = leftSide.contains(from) ? rightSide : leftSide;
            for (const Edge edge : graph.edgesFrom(from)) {
                const Vertex to = graph.get(ToVertex, edge);
                if (otherSide.contains(to))
                    continue;
                otherSide.insert(to);
                unassignedVertices.emplace_back(to);
            }
        }
    }

    inline bool checkSideContainsOnlyVerticesInGraph(IndexedSet<false, Vertex>& side)
    {
        for (auto& vertex : side.getValues()) {
            if (!graph.isVertex(vertex))
                return false;
        }
        return true;
    }

public:
    DynamicFlowGraph graph;
    IndexedSet<false, Vertex> leftSide;
    IndexedSet<false, Vertex> rightSide;
};

inline std::vector<Dinic::CutEdge> maximumBipartiteMatching(BipartiteGraph& data)
{
    const Vertex source = data.graph.addVertex();
    for (const Vertex to : data.leftSide) {
        data.graph.addEdge(source, to).set(Capacity, 1);
        data.graph.addEdge(to, source).set(Capacity, 0);
    }
    const Vertex target = data.graph.addVertex();
    for (const Vertex from : data.rightSide) {
        for (const Edge edge : data.graph.edgesFrom(from)) {
            data.graph.set(Capacity, edge, 0);
        }
        data.graph.addEdge(from, target).set(Capacity, 1);
        data.graph.addEdge(target, from).set(Capacity, 0);
    }

    Dinic dinic(data.graph);
    dinic.run(source, target);
    std::vector<Dinic::CutEdge> result;
    const StaticFlowGraph& flowGraph = dinic.getFlowGraph();
    for (const Vertex from : data.leftSide) {
        for (const Edge edge : flowGraph.edgesFrom(from)) {
            if (dinic.getFlow(edge) != 1)
                continue;
            const Vertex to = flowGraph.get(ToVertex, edge);
            if (!data.rightSide.contains(to))
                continue;
            result.emplace_back(from, to, edge);
        }
    }
    return result;
}

inline std::vector<Dinic::CutEdge> maximumBipartiteMatching(DynamicFlowGraph& graph)
{
    const Vertex source = graph.addVertex();
    const Vertex target = graph.addVertex();

    for (size_t i(0); i < (graph.numVertices() - 2); i += 2) {
        graph.addEdge(source, Vertex(i)).set(Capacity, 1);
        graph.addEdge(Vertex(i), source).set(Capacity, 0);

        graph.addEdge(Vertex(i + 1), target).set(Capacity, 1);
        graph.addEdge(target, Vertex(i + 1)).set(Capacity, 0);
    }

    Dinic dinic(graph);
    dinic.run(source, target);
    std::vector<Dinic::CutEdge> result;
    const StaticFlowGraph& flowGraph = dinic.getFlowGraph();
    for (size_t i(0); i < (flowGraph.numVertices() - 2); i += 2) {
        const Vertex from(i);
        for (const Edge edge : flowGraph.edgesFrom(from)) {
            if (dinic.getFlow(edge) != 1)
                continue;
            const Vertex to = flowGraph.get(ToVertex, edge);
            if (!((to & 1) && to != target))
                continue;
            result.emplace_back(from, to, edge);
        }
    }
    return result;
}

template <typename GRAPH>
inline std::vector<Dinic::CutEdge> maximumBipartiteMatching(const GRAPH& graph)
{
    BipartiteGraph data(graph);
    return maximumBipartiteMatching(data);
}

template <typename GRAPH>
inline std::vector<Vertex> minimumBipartiteVertexCover(const GRAPH& graph)
{
    BipartiteGraph data(graph);
    DynamicFlowGraph matchingGraph;
    matchingGraph.addVertices(data.graph.numVertices());
    for (const Vertex from : data.leftSide) {
        for (const Edge edge : data.graph.edgesFrom(from)) {
            matchingGraph.addEdge(from, data.graph.get(ToVertex, edge));
        }
    }
    IndexedSet<false, Vertex> connectedVertices = data.leftSide;
    for (const Dinic::CutEdge edge : maximumBipartiteMatching(data)) {
        AssertMsg(data.leftSide.contains(edge.from), "Matching edge is not directed from left to right side!");
        AssertMsg(data.rightSide.contains(edge.to), "Matching edge is not directed from left to right side!");
        matchingGraph.deleteEdge(edge.from, edge.to);
        matchingGraph.addEdge(edge.to, edge.from);
        connectedVertices.remove(edge.from);
    }
    std::vector<Vertex> unprocessedVertices;
    for (const Vertex vertex : connectedVertices) {
        unprocessedVertices.emplace_back(vertex);
    }
    while (!unprocessedVertices.empty()) {
        const Vertex from = unprocessedVertices.back();
        unprocessedVertices.pop_back();
        for (const Edge edge : matchingGraph.edgesFrom(from)) {
            const Vertex to = matchingGraph.get(ToVertex, edge);
            if (connectedVertices.contains(to))
                continue;
            connectedVertices.insert(to);
            unprocessedVertices.emplace_back(to);
        }
    }
    std::vector<Vertex> result;
    for (const Vertex vertex : data.leftSide) {
        if (connectedVertices.contains(vertex))
            continue;
        result.emplace_back(vertex);
    }
    for (const Vertex vertex : data.rightSide) {
        if (!connectedVertices.contains(vertex))
            continue;
        result.emplace_back(vertex);
    }
    return result;
}
