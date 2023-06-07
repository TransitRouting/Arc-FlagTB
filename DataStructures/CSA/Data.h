#pragma once

#include <algorithm>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#include "../../Algorithms/Dijkstra/Dijkstra.h"
#include "../../DataStructures/Container/Map.h"
#include "../../Helpers/Console/Progress.h"
#include "../../Helpers/Console/ProgressBar.h"
#include "../../Helpers/FileSystem/FileSystem.h"
#include "../../Helpers/IO/ParserCSV.h"
#include "../../Helpers/IO/Serialization.h"
#include "../../Helpers/Ranges/Range.h"
#include "../../Helpers/String/String.h"
#include "../../Helpers/Vector/Permutation.h"
#include "../Geometry/Rectangle.h"
#include "../Graph/Graph.h"
#include "../Intermediate/Data.h"
#include "Entities/Connection.h"
#include "Entities/Journey.h"
#include "Entities/Stop.h"
#include "Entities/Trip.h"

namespace CSA {

using TransferGraph = ::TransferGraph;

class Data {
private:
    Data()
    {
    }

public:
    Data(const std::string& fileName)
    {
        deserialize(fileName);
    }

    inline static Data FromBinary(const std::string& fileName) noexcept
    {
        Data data;
        data.deserialize(fileName);
        return data;
    }

    inline static Data FromIntermediate(const Intermediate::Data& inter) noexcept
    {
        Data data;
        for (const Intermediate::Stop& stop : inter.stops) {
            data.stopData.emplace_back(stop);
        }
        for (const Intermediate::Trip& trip : inter.trips) {
            AssertMsg(!trip.stopEvents.empty(), "Intermediate data contains trip without any stop event!");
            for (size_t i = 1; i < trip.stopEvents.size(); i++) {
                const Intermediate::StopEvent& from = trip.stopEvents[i - 1];
                const Intermediate::StopEvent& to = trip.stopEvents[i];
                data.connections.emplace_back(from.stopId, to.stopId, from.departureTime, to.arrivalTime,
                    TripId(data.tripData.size()));
            }
            data.tripData.emplace_back(trip);
        }
        std::sort(data.connections.begin(), data.connections.end());
        Intermediate::TransferGraph transferGraph = inter.transferGraph;
        Graph::printInfo(transferGraph);
        transferGraph.printAnalysis();
        Graph::move(std::move(transferGraph), data.transferGraph);
        return data;
    }

    template <bool MAKE_BIDIRECTIONAL = true, typename GRAPH_TYPE>
    inline static Data FromInput(const std::vector<Stop>& stops, const std::vector<Connection>& connections,
        const std::vector<Trip>& trips, GRAPH_TYPE transferGraph) noexcept
    {
        AssertMsg(transferGraph.numVertices() >= stops.size(),
            "Network containes " << stops.size() << " stops, but transfer graph has only "
                                 << transferGraph.numVertices() << " vertices!");
        Data data;
        data.stopData = stops;
        data.tripData = trips;
        for (const Connection con : connections) {
            if (con.departureStopId >= data.stopData.size() || data.stopData[con.departureStopId].minTransferTime < 0)
                continue;
            if (con.arrivalStopId >= data.stopData.size() || data.stopData[con.arrivalStopId].minTransferTime < 0)
                continue;
            if (con.departureTime > con.arrivalTime)
                continue;
            if (con.tripId >= data.tripData.size())
                continue;
            data.connections.emplace_back(con);
        }
        Intermediate::TransferGraph graph;
        Graph::move(std::move(transferGraph), graph);
        if constexpr (MAKE_BIDIRECTIONAL)
            graph.makeBidirectional();
        graph.reduceMultiEdgesBy(TravelTime);
        graph.packEdges();
        Graph::move(std::move(graph), data.transferGraph);
        return data;
    }

public:
    inline size_t numberOfStops() const noexcept
    {
        return stopData.size();
    }
    inline bool isStop(const Vertex stop) const noexcept
    {
        return stop < numberOfStops();
    }
    inline Range<StopId> stops() const noexcept
    {
        return Range<StopId>(StopId(0), StopId(numberOfStops()));
    }

    inline size_t numberOfTrips() const noexcept
    {
        return tripData.size();
    }
    inline bool isTrip(const TripId tripId) const noexcept
    {
        return tripId < numberOfTrips();
    }
    inline Range<TripId> tripIds() const noexcept
    {
        return Range<TripId>(TripId(0), TripId(numberOfTrips()));
    }

    inline size_t numberOfConnections() const noexcept
    {
        return connections.size();
    }
    inline bool isConnection(const ConnectionId connectionId) const noexcept
    {
        return connectionId < numberOfConnections();
    }
    inline Range<ConnectionId> connectionIds() const noexcept
    {
        return Range<ConnectionId>(ConnectionId(0), ConnectionId(numberOfConnections()));
    }

    inline int minTransferTime(const StopId stop) const noexcept
    {
        return stopData[stop].minTransferTime;
    }

    inline Geometry::Rectangle boundingBox() const noexcept
    {
        Geometry::Rectangle result = Geometry::Rectangle::Empty();
        for (const Stop& stop : stopData) {
            result.extend(stop.coordinates);
        }
        return result;
    }

    inline void sortConnectionsAscending() noexcept
    {
        std::stable_sort(connections.begin(), connections.end());
    }

    inline void sortConnectionsDescending() noexcept
    {
        std::stable_sort(connections.begin(), connections.end(),
            [](const Connection& a, const Connection& b) { return b < a; });
    }

    inline void sortConnectionsAscendingByDepartureTime() noexcept
    {
        std::stable_sort(connections.begin(), connections.end(),
            [](const Connection& a, const Connection& b) { return a.departureTime < b.departureTime; });
    }

    inline void sortConnectionsAscendingByArrivalTime() noexcept
    {
        std::stable_sort(connections.begin(), connections.end(),
            [](const Connection& a, const Connection& b) { return a.arrivalTime < b.arrivalTime; });
    }

    inline void sortConnectionsDescendingByDepartureTime() noexcept
    {
        std::stable_sort(connections.begin(), connections.end(),
            [](const Connection& a, const Connection& b) { return a.departureTime > b.departureTime; });
    }

    inline void sortConnectionsDescendingByArrivalTime() noexcept
    {
        std::stable_sort(connections.begin(), connections.end(),
            [](const Connection& a, const Connection& b) { return a.arrivalTime > b.arrivalTime; });
    }

    inline void sortUnique() noexcept
    {
        std::sort(connections.begin(), connections.end(), [](const Connection& a, const Connection& b) {
            return std::make_tuple(a.departureTime, a.arrivalTime, a.departureStopId, a.arrivalStopId, a.tripId) < std::make_tuple(b.departureTime, b.arrivalTime, b.departureStopId, b.arrivalStopId, b.tripId);
        });
    }

    inline bool isCombinable(const Vertex source, const int departureTime, const Vertex target,
        const int arrivalTime = intMax) const noexcept
    {
        AssertMsg(transferGraph.isVertex(source), "Source vertex id " << source << " does not represent a vertex!");
        AssertMsg(transferGraph.isVertex(target), "Target vertex id " << target << " does not represent a vertex!");
        if (source == target) {
            return departureTime <= arrivalTime;
        } else {
            Edge transferEdge = transferGraph.findEdge(source, target);
            if (!transferGraph.isEdge(transferEdge))
                return false;
            return departureTime + transferGraph.get(TravelTime, transferEdge) <= arrivalTime;
        }
    }

    template <bool APPLY_MIN_TRANSFER_TIME>
    inline bool isCombinable(const StopId source, const int departureTime, const StopId target,
        const int arrivalTime = intMax) const noexcept
    {
        AssertMsg(isStop(source), "Source vertex id " << source << " does not represent a stop!");
        AssertMsg(isStop(target), "Target vertex id " << target << " does not represent a stop!");
        if constexpr (APPLY_MIN_TRANSFER_TIME) {
            if (source == target) {
                return departureTime + minTransferTime(source) <= arrivalTime;
            }
        }
        return isCombinable(source, departureTime, target, arrivalTime);
    }

    inline bool isCombinable(const Connection& first, const Connection& second) const noexcept
    {
        if (first.arrivalTime > second.departureTime)
            return false;
        if (first.tripId == second.tripId)
            return true;
        return isCombinable<true>(first.arrivalStopId, first.arrivalTime, second.departureStopId, second.departureTime);
    }

    inline bool isCombinable(const Vertex source, const int departureTime, const Connection& second) const noexcept
    {
        return isCombinable(source, departureTime, second.departureStopId, second.departureTime);
    }

    template <bool APPLY_MIN_TRANSFER_TIME>
    inline bool isCombinable(const StopId source, const int departureTime, const Connection& second) const noexcept
    {
        return isCombinable<APPLY_MIN_TRANSFER_TIME>(source, departureTime, second.departureStopId,
            second.departureTime);
    }

    inline bool isCombinable(const Connection& first, const StopId target,
        const int arrivalTime = intMax) const noexcept
    {
        return isCombinable<true>(first.arrivalStopId, first.arrivalTime, target, arrivalTime);
    }

    inline bool isCombinable(const Connection& first, const Vertex target,
        const int arrivalTime = intMax) const noexcept
    {
        return isCombinable(first.arrivalStopId, first.arrivalTime, target, arrivalTime);
    }

    inline void makeUndirectedTransitiveStopGraph(const bool verbose = false) noexcept
    {
        Intermediate::TransferGraph graph;
        graph.addVertices(transferGraph.numVertices());
        for (const Vertex from : transferGraph.vertices()) {
            graph.set(Coordinates, from, transferGraph.get(Coordinates, from));
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                const Vertex to = transferGraph.get(ToVertex, edge);
                if (to >= stopData.size())
                    continue;
                graph.addEdge(from, to).set(TravelTime, transferGraph.get(TravelTime, edge));
            }
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
        graph.clear();
        graph.addVertices(transferGraph.numVertices());
        Dijkstra<TransferGraph, false> dijkstra(transferGraph, transferGraph[TravelTime]);
        Progress progress(transferGraph.numVertices(), verbose);
        for (const Vertex v : transferGraph.vertices()) {
            graph.set(Coordinates, v, transferGraph.get(Coordinates, v));
            dijkstra.run(v, noVertex, [&](const Vertex u) {
                if (u >= v)
                    return;
                const int travelTime = dijkstra.getDistance(u);
                graph.addEdge(v, u).set(TravelTime, travelTime);
                graph.addEdge(u, v).set(TravelTime, travelTime);
            });
            progress++;
        }
        graph.packEdges();
        Graph::move(std::move(graph), transferGraph);
    }

    inline void makeDirectedTransitiveStopGraph(const bool verbose = false) noexcept
    {
        Intermediate::TransferGraph toZones;
        Intermediate::TransferGraph fromZones;
        Intermediate::TransferGraph newTransferGraph;
        toZones.addVertices(transferGraph.numVertices());
        fromZones.addVertices(transferGraph.numVertices());
        newTransferGraph.addVertices(transferGraph.numVertices());
        for (const Vertex from : transferGraph.vertices()) {
            newTransferGraph.set(Coordinates, from, transferGraph.get(Coordinates, from));
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                const Vertex to = transferGraph.get(ToVertex, edge);
                const int travelTime = transferGraph.get(TravelTime, edge);
                if (from < stopData.size())
                    toZones.addEdge(from, to).set(TravelTime, travelTime);
                if (to < stopData.size())
                    fromZones.addEdge(from, to).set(TravelTime, travelTime);
            }
        }
        if (stopData.size() > 0) {
            toZones.packEdges();
            Dijkstra<Intermediate::TransferGraph, false> dijkstra(toZones, toZones[TravelTime]);
            Progress progress(stopData.size(), verbose);
            for (const StopId stop : stops()) {
                dijkstra.run(stop, noVertex, [&](const Vertex u) {
                    if (u != stop)
                        newTransferGraph.addEdge(stop, u).set(TravelTime, dijkstra.getDistance(u));
                });
                progress++;
            }
        }
        if (transferGraph.numVertices() > stopData.size()) {
            fromZones.packEdges();
            Dijkstra<Intermediate::TransferGraph, false> dijkstra(fromZones, fromZones[TravelTime]);
            Progress progress(transferGraph.numVertices() - stopData.size(), verbose);
            for (Vertex v = Vertex(stopData.size()); v < transferGraph.numVertices(); v++) {
                dijkstra.run(v, noVertex, [&](const Vertex u) {
                    if (u != v)
                        newTransferGraph.addEdge(v, u).set(TravelTime, dijkstra.getDistance(u));
                });
                progress++;
            }
        }
        Dijkstra<TransferGraph, false> dijkstraToZones(transferGraph, transferGraph[TravelTime]);
        newTransferGraph.packEdges();
        Graph::move(std::move(newTransferGraph), transferGraph);
    }

    inline void duplicateConnections(const int timeOffset = 24 * 60 * 60) noexcept
    {
        const size_t oldConnectionCount = connections.size();
        const size_t oldTripCount = tripData.size();
        for (size_t i = 0; i < oldConnectionCount; i++) {
            connections.emplace_back(connections[i], timeOffset, oldTripCount);
        }
        for (size_t i = 0; i < oldTripCount; i++) {
            tripData.emplace_back(tripData[i]);
        }
    }

    inline std::vector<int> numberOfNeighborStopsByStop() const noexcept
    {
        std::vector<int> result(numberOfStops(), 0);
        for (const StopId stop : stops()) {
            for (const Edge edge : transferGraph.edgesFrom(stop)) {
                if (isStop(transferGraph.get(ToVertex, edge))) {
                    result[stop]++;
                }
            }
        }
        return result;
    }

    inline void applyMinTravelTime(const double minTravelTime) noexcept
    {
        for (const Vertex from : transferGraph.vertices()) {
            for (const Edge edge : transferGraph.edgesFrom(from)) {
                if (transferGraph.get(TravelTime, edge) < minTravelTime) {
                    transferGraph.set(TravelTime, edge, minTravelTime);
                }
            }
        }
    }

    inline void applyVertexPermutation(const Permutation& permutation, const bool permutateStops = true) noexcept
    {
        Permutation splitPermutation = permutation.splitAt(numberOfStops());
        if (!permutateStops) {
            for (size_t i = 0; i < numberOfStops(); i++) {
                splitPermutation[i] = i;
            }
        }
        Permutation stopPermutation = splitPermutation;
        stopPermutation.resize(numberOfStops());
        permutate(splitPermutation, stopPermutation);
    }

    inline void applyVertexOrder(const Order& order, const bool permutateStops = true) noexcept
    {
        applyVertexPermutation(Permutation(Construct::Invert, order), permutateStops);
    }

    inline void applyStopPermutation(const Permutation& permutation) noexcept
    {
        permutate(permutation.extend(transferGraph.numVertices()), permutation);
    }

    inline void applyStopOrder(const Order& order) noexcept
    {
        applyStopPermutation(Permutation(Construct::Invert, order));
    }

public:
    inline const std::vector<Geometry::Point>& getCoordinates() const noexcept
    {
        return transferGraph[Coordinates];
    }

    inline std::string journeyToShortText(const std::vector<ConnectionId>& connectionList) const noexcept
    {
        if (connectionList.empty())
            return "";
        std::stringstream text;
        TripId trip = connections[connectionList.front()].tripId;
        text << stopData[connections[connectionList.front()].departureStopId].name << "["
             << connections[connectionList.front()].departureStopId << "] -> ";
        text << tripData[trip].tripName << "[" << trip << "] -> ";
        for (size_t i = 1; i < connectionList.size(); i++) {
            if (connections[connectionList[i]].tripId == trip)
                continue;
            trip = connections[connectionList[i]].tripId;
            text << stopData[connections[connectionList[i - 1]].arrivalStopId].name << "["
                 << connections[connectionList[i - 1]].arrivalStopId << "] -> ";
            if (connections[connectionList[i]].departureStopId != connections[connectionList[i - 1]].arrivalStopId) {
                text << stopData[connections[connectionList[i]].departureStopId].name << "["
                     << connections[connectionList[i]].departureStopId << "] -> ";
            }
            text << tripData[trip].tripName << "[" << trip << "] -> ";
        }
        text << stopData[connections[connectionList.back()].arrivalStopId].name << "["
             << connections[connectionList.front()].arrivalStopId << "]";
        return text.str();
    }

    inline std::vector<std::string> journeyToText(const Journey& journey) const noexcept
    {
        std::vector<std::string> text;
        for (const JourneyLeg& leg : journey) {
            std::stringstream line;
            if (leg.usesTrip) {
                line << "Take " << GTFS::TypeNames[tripData[leg.tripId].type];
                line << ": " << tripData[leg.tripId].routeName << "(" << tripData[leg.tripId].tripName << ")"
                     << "[" << leg.tripId << "] ";
                line << "from " << stopData[leg.from].name << "[" << leg.from << "] ";
                line << "departing at " << String::secToTime(leg.departureTime) << "[" << leg.departureTime << "] ";
                line << "to " << stopData[leg.to].name << "[" << leg.to << "] ";
                line << "arrive at " << String::secToTime(leg.arrivalTime) << "[" << leg.arrivalTime << "];";
            } else if (leg.from == leg.to) {
                line << "Wait at " << stopData[leg.from].name << " [" << leg.from << "], ";
                line << "minimal waiting time: " << String::secToString(leg.arrivalTime - leg.departureTime) << ".";
            } else {
                line << "Walk from " << (isStop(leg.from) ? stopData[leg.from].name : "Vertex") << " [" << leg.from
                     << "] ";
                line << "to " << (isStop(leg.to) ? stopData[leg.to].name : "Vertex") << " [" << leg.to << "], ";
                line << "start at " << String::secToTime(leg.departureTime) << " [" << leg.departureTime << "] ";
                line << "and arrive at " << String::secToTime(leg.arrivalTime) << " [" << leg.arrivalTime << "] ";
                line << "(" << String::secToString(leg.arrivalTime - leg.departureTime) << ").";
            }
            text.emplace_back(line.str());
        }
        return text;
    }

    inline std::string journeyToText(const std::vector<ConnectionId>& connectionList) const noexcept
    {
        std::stringstream text;
        TripId currentTripIndex = TripId(0);
        TripId nextTripIndex = TripId(0);
        while (currentTripIndex < connectionList.size()) {
            while (nextTripIndex < connectionList.size() && connections[connectionList[currentTripIndex]].tripId == connections[connectionList[nextTripIndex]].tripId) {
                nextTripIndex++;
            }
            Connection c1 = connections[connectionList[currentTripIndex]];
            Connection c2 = connections[connectionList[nextTripIndex - 1]];
            text << "Take " << GTFS::TypeNames[tripData[c1.tripId].type];
            text << ": " << tripData[c1.tripId].routeName << "(" << tripData[c1.tripId].tripName << ")"
                 << "[" << c1.tripId << "] ";
            text << "from " << stopData[c1.departureStopId].name << "[" << c1.departureStopId << "] ";
            text << "departing at " << String::secToTime(c1.departureTime) << "[" << c1.departureTime << "] ";
            text << "to " << stopData[c2.arrivalStopId].name << "[" << c2.arrivalStopId << "] ";
            text << "arrive at " << String::secToTime(c2.arrivalTime) << "[" << c2.arrivalTime << "];";
            if (nextTripIndex < connectionList.size()) {
                text << " ";
                Connection c3 = connections[connectionList[nextTripIndex]];
                if (c2.arrivalStopId == c3.departureStopId) {
                    text << "Wait at " << stopData[c2.arrivalStopId].name << " [" << c2.arrivalStopId << "], ";
                    text << "minimal waiting time: " << String::secToString(c3.departureTime - c2.arrivalTime) << ".";
                } else {
                    text << "Walk from " << stopData[c2.arrivalStopId].name << " [" << c2.arrivalStopId << "] ";
                    text << "to " << stopData[c3.departureStopId].name << " [" << c3.departureStopId << "], ";
                    text << "start at " << String::secToTime(c2.arrivalTime) << " [" << c2.arrivalTime << "] ";
                    text << "and arrive at " << String::secToTime(c3.departureTime) << " [" << c3.departureTime << "] ";
                    text << "(" << String::secToString(c3.departureTime - c2.arrivalTime) << ").";
                }
            }
            currentTripIndex = nextTripIndex;
        }
        return text.str();
    }

    inline TransferGraph minTravelTimeGraph() const noexcept
    {
        Intermediate::TransferGraph topology;
        topology.addVertices(transferGraph.numVertices());
        for (const Connection& connection : connections) {
            if (connection.departureStopId == connection.arrivalStopId)
                continue;
            const size_t numEdges = topology.numEdges();
            const Edge newEdge = topology.findOrAddEdge(connection.departureStopId, connection.arrivalStopId);
            if (topology.numEdges() != numEdges) {
                topology.set(TravelTime, newEdge, intMax);
            }
            topology.set(
                TravelTime, newEdge,
                std::min(topology.get(TravelTime, newEdge), connection.arrivalTime - connection.departureTime));
        }
        for (Vertex vertex : transferGraph.vertices()) {
            topology.set(Coordinates, vertex, transferGraph.get(Coordinates, vertex));
            for (Edge edge : transferGraph.edgesFrom(vertex)) {
                if (vertex == transferGraph.get(ToVertex, edge))
                    continue;
                const size_t numEdges = topology.numEdges();
                const Edge newEdge = topology.findOrAddEdge(vertex, transferGraph.get(ToVertex, edge));
                if (topology.numEdges() != numEdges) {
                    topology.set(TravelTime, newEdge, intMax);
                }
                topology.set(TravelTime, newEdge,
                    std::min(topology.get(TravelTime, newEdge), transferGraph.get(TravelTime, edge)));
            }
        }
        topology.deleteEdges([&](Edge edge) { return topology.get(TravelTime, edge) >= intMax; });
        topology.packEdges();
        TransferGraph result;
        Graph::move(std::move(topology), result);
        return result;
    }

    inline void printInfo() const noexcept
    {
        int firstDay = std::numeric_limits<int>::max();
        int lastDay = std::numeric_limits<int>::min();
        std::vector<int> departuresByStop(numberOfStops(), 0);
        std::vector<int> arrivalsByStop(numberOfStops(), 0);
        std::vector<int> connectionsByStop(numberOfStops(), 0);
        for (const Connection& connection : connections) {
            if (firstDay > connection.departureTime)
                firstDay = connection.departureTime;
            if (lastDay < connection.arrivalTime)
                lastDay = connection.arrivalTime;
            departuresByStop[connection.departureStopId]++;
            arrivalsByStop[connection.arrivalStopId]++;
            connectionsByStop[connection.departureStopId]++;
            connectionsByStop[connection.arrivalStopId]++;
        }
        size_t numberOfIsolatedStops = 0;
        for (const StopId stop : stops()) {
            if (transferGraph.outDegree(stop) == 0)
                numberOfIsolatedStops++;
        }
        std::cout << "CSA public transit data:" << std::endl;
        std::cout << "   Number of Stops:           " << std::setw(12) << String::prettyInt(numberOfStops())
                  << std::endl;
        std::cout << "   Number of Isolated Stops:  " << std::setw(12) << String::prettyInt(numberOfIsolatedStops)
                  << std::endl;
        std::cout << "   Number of Trips:           " << std::setw(12) << String::prettyInt(numberOfTrips())
                  << std::endl;
        std::cout << "   Number of Stop Events:     " << std::setw(12)
                  << String::prettyInt(numberOfConnections() + numberOfTrips()) << std::endl;
        std::cout << "   Number of Connections:     " << std::setw(12) << String::prettyInt(numberOfConnections())
                  << std::endl;
        std::cout << "   Number of Vertices:        " << std::setw(12) << String::prettyInt(transferGraph.numVertices())
                  << std::endl;
        std::cout << "   Number of Edges:           " << std::setw(12) << String::prettyInt(transferGraph.numEdges())
                  << std::endl;
        std::cout << "   Stops without departures:  " << std::setw(12)
                  << String::prettyInt(Vector::count(departuresByStop, 0)) << std::endl;
        std::cout << "   Stops without arrivals:    " << std::setw(12)
                  << String::prettyInt(Vector::count(arrivalsByStop, 0)) << std::endl;
        std::cout << "   Stops without connections: " << std::setw(12)
                  << String::prettyInt(Vector::count(connectionsByStop, 0)) << std::endl;
        std::cout << "   First Day:                 " << std::setw(12) << String::prettyInt(firstDay / (60 * 60 * 24))
                  << std::endl;
        std::cout << "   Last Day:                  " << std::setw(12) << String::prettyInt(lastDay / (60 * 60 * 24))
                  << std::endl;
        std::cout << "   First Departure:           " << std::setw(12) << String::secToTime(firstDay) << std::endl;
        std::cout << "   Last Arrival:              " << std::setw(12) << String::secToTime(lastDay) << std::endl;
        std::cout << "   Bounding Box:              " << std::setw(12) << boundingBox() << std::endl;
        /*std::cout << "   Transfer graph:            " << std::setw(12) <<
        Graph::characterize(transferGraph) << std::endl; if
        (transferGraph.numVertices() > numberOfStops()) { DynamicTransferGraph
        stopGraph; Graph::copy(transferGraph, stopGraph);
            stopGraph.deleteVertices([&](const Vertex v){return v >=
        numberOfStops();}); std::cout << "   Stop graph:                " <<
        std::setw(12) << Graph::characterize(stopGraph) << std::endl;
        }*/
    }

    inline void serialize(const std::string& fileName) const noexcept
    {
        IO::serialize(fileName, connections, stopData, tripData); //, adjTrip, adjStop, connectionsStop, connectionsTrip);
        transferGraph.writeBinary(fileName + ".graph");
    }

    inline void deserialize(const std::string& fileName) noexcept
    {
        IO::deserialize(fileName, connections, stopData, tripData); //, adjTrip, adjStop, connectionsStop, connectionsTrip);
        transferGraph.readBinary(fileName + ".graph");
    }

    inline void createGraphForMETIS(const bool verbose = true) noexcept
    {
        layoutGraph.clear();
        layoutGraph.addVertices(stopData.size());

        for (Vertex vertex : layoutGraph.vertices())
            layoutGraph.set(Weight, vertex, 1);

        size_t amountOfWork = numberOfConnections() + transferGraph.numEdges();

        Progress progCreatingGraph(amountOfWork);

        for (auto& connection : connections) {
            Vertex from(connection.departureStopId);
            Vertex to(connection.arrivalStopId);

            if (from == to)
                continue;
            Edge edgeHeadTail = layoutGraph.findEdge(from, to);

            if (edgeHeadTail != noEdge) {
                layoutGraph.set(Weight, edgeHeadTail, layoutGraph.get(Weight, edgeHeadTail) + 1);
                Edge edgeTailHead = layoutGraph.findEdge(to, from);
                AssertMsg(edgeTailHead != noEdge, "A reverse edge is missing!\n");
                layoutGraph.set(Weight, edgeTailHead, layoutGraph.get(Weight, edgeTailHead) + 1);
            } else {
                layoutGraph.addEdge(from, to).set(Weight, 1);
                layoutGraph.addEdge(to, from).set(Weight, 1);
            }

            progCreatingGraph++;
        }

        for (const auto [transferEdge, from] : transferGraph.edgesWithFromVertex()) {
            Vertex to = transferGraph.get(ToVertex, transferEdge);
            if (to == Vertex(from))
                continue;
            AssertMsg(layoutGraph.isVertex(from), "from Vertex is not a valid Vertex!\n");
            AssertMsg(layoutGraph.isVertex(to), "to Vertex is not a valid Vertex!\n");
            Edge edge = layoutGraph.findEdge(Vertex(from), to);
            if (edge == noEdge) {
                layoutGraph.addEdge(Vertex(from), to).set(Weight, INFTY);
                layoutGraph.addEdge(to, Vertex(from)).set(Weight, INFTY);
            }

            progCreatingGraph++;
        }

        progCreatingGraph.finished();
        AssertMsg(!(layoutGraph.edges().size() & 1), "The number of edges is uneven, thus we check that every edge "
                                                     "has a reverse edge in the graph!\n");
        if (verbose)
            std::cout << "Graph has been created!\nNumber of vertices:\t" << layoutGraph.numVertices()
                      << "\nNumber of edges:\t" << layoutGraph.edges().size() << "\n";
    }

    inline void writeMETISFile(const std::string& fileName, const bool verbose = true) noexcept
    {
        Progress progWritingMETIS(stopData.size());

        unsigned long n = layoutGraph.numVertices();
        unsigned long m = layoutGraph.numEdges() >> 1; // halbieren

        std::ofstream metisFile(fileName);

        // n [NUMBER of nodes]  m [NUMBER of edges]     f [int]
        // f values:
        /*
                f values:
        1 :     edge-weighted graph
        10:     node-weighted graph
        11:     edge & node - weighted graph
         */

        metisFile << n << " " << m << " 11";

        for (StopId stop(0); stop < numberOfStops(); ++stop) {
            metisFile << "\n"
                      << layoutGraph.get(Weight, Vertex(stop)) << " ";
            for (Edge edge : layoutGraph.edgesFrom(Vertex(stop))) {
                metisFile << layoutGraph.get(ToVertex, edge).value() + 1 << " " << layoutGraph.get(Weight, edge) << " ";
            }
            progWritingMETIS++;
        }
        metisFile.close();
        progWritingMETIS.finished();

        if (verbose)
            std::cout << "Finished creating metis file " << fileName << "\n";
    }

    /*
    // Create Additional Journey Datastructures
    inline void createJourneyExtrDatastructures(const bool verbose = true) noexcept
    {
        if (verbose)
            std::cout << "Computing the additional datastructures needed for faster journey extraction ...\n";

        adjTrip.assign(numberOfTrips() + 1, 0);
        adjStop.assign(numberOfStops() + 1, 0);
        connectionsTrip.assign(numberOfConnections(), ConnectionId(0));
        connectionsStop.assign(numberOfConnections(), ConnectionId(0));

        // compute number of connections passing through stop
        for (auto& connection : connections) {
            ++adjTrip[connection.tripId];
            ++adjStop[connection.arrivalStopId];
        }

        // prefix sums (for adjStop and adjTrip)

        int sum(0);
        for (size_t i(0); i < adjStop.size(); ++i) {
            int tmp = adjStop[i];
            adjStop[i] = sum;
            sum += tmp;
        }

        Assert((size_t)adjStop[0] == (size_t)0);
        Assert((size_t)adjStop[numberOfStops()] == (size_t)numberOfConnections());

        sum = 0;
        for (size_t i(0); i < adjTrip.size(); ++i) {
            int tmp = adjTrip[i];
            adjTrip[i] = sum;
            sum += tmp;
        }

        Assert((size_t)adjTrip[0] == (size_t)0);
        Assert((size_t)adjTrip[numberOfTrips()] == (size_t)numberOfConnections());

        // Put connections into correct bucket

        AssertMsg(Vector::isSorted(connections), "Connections must be sorted in ascending order!");

        std::vector<int> stopOffset(numberOfStops(), 0);
        std::vector<int> tripOffset(numberOfTrips(), 0);

        Progress progress(numberOfConnections() + adjStop.size());

        for (ConnectionId i(0); i < numberOfConnections(); ++i) {
            auto currentTripId = connections[i].tripId;
            auto currentStopId = connections[i].arrivalStopId;

            int tripPositionToInsert = adjTrip[currentTripId] + tripOffset[currentTripId];
            int stopPositionToInsert = adjStop[currentStopId] + stopOffset[currentStopId];

            ++tripOffset[currentTripId];
            ++stopOffset[currentStopId];

            connectionsTrip[tripPositionToInsert] = i;
            connectionsStop[stopPositionToInsert] = i;
            progress++;
        }

        // sort the connectionIds in connectionsStop by arrivalTime

        for (size_t i(0); i < adjStop.size() - 1; ++i) {
            std::stable_sort(connectionsStop.begin() + adjStop[i], connectionsStop.begin() + adjStop[i + 1], [this](const auto& lhs, const auto& rhs) {
                return connections[lhs].arrivalTime < connections[rhs].arrivalTime;
            });
            progress++;
        }
        progress.finished();

        if (verbose)
            std::cout << "Finished creating the datastructures!\n";
    }
*/

private:
    inline void permutate(const Permutation& fullPermutation, const Permutation& stopPermutation) noexcept
    {
        AssertMsg(fullPermutation.size() == transferGraph.numVertices(),
            "Full permutation size (" << fullPermutation.size() << ") must be the same as number of vertices ("
                                      << transferGraph.numVertices() << ")!");
        AssertMsg(stopPermutation.size() == numberOfStops(),
            "Stop permutation size (" << stopPermutation.size() << ") must be the same as number of stops ("
                                      << numberOfStops() << ")!");

        for (Connection& connection : connections) {
            connection.applyStopPermutation(stopPermutation);
        }
        stopPermutation.permutate(stopData);

        transferGraph.applyVertexPermutation(fullPermutation);
    }

public:
    std::vector<Connection> connections;
    std::vector<Stop> stopData;
    std::vector<Trip> tripData;

    TransferGraph transferGraph;

    DynamicGraphWithWeights layoutGraph;

    // Journey Extraction - Additional Datastructures
    /*
    std::vector<int> adjTrip;
    std::vector<ConnectionId> connectionsTrip;
    std::vector<int> adjStop;
    std::vector<ConnectionId> connectionsStop;
    */
};

} // namespace CSA
