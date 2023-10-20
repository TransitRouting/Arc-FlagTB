#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/TransferPattern/Preprocessing/TransferPatternBuilder.h"
#include "../../Algorithms/TransferPattern/Query/Query.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/Queries/Queries.h"
#include "../../DataStructures/TransferPattern/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class RunTransferPatternQueries : public ParameterizedCommand {
public:
    RunTransferPatternQueries(BasicShell& shell)
        : ParameterizedCommand(shell, "runTPQueries", "Runs the given number of random Transfer Pattern Queries.")
    {
        addParameter("Input file (TP Data)");
        addParameter("Number of queries");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (TP Data)");

        TransferPattern::Data data(inputFile);
        data.printInfo();

        TransferPattern::Query<TransferPattern::AggregateProfiler> algorithm(data);

        const size_t n = getParameter<size_t>("Number of queries");
        const std::vector<StopQuery> queries = generateRandomStopQueries(data.raptorData.numberOfStops(), n);
        /* const std::vector<StopQuery> queries = { */
        /*     StopQuery(StopId(617), StopId(289), 72170), */
        /*     StopQuery(StopId(1829), StopId(1532), 32360), */
        /*     StopQuery(StopId(300), StopId(1148), 51724) */
        /* }; */
        double numJourneys = 0;
        for (const StopQuery& query : queries) {
            algorithm.run(query.source, query.departureTime, query.target);
            numJourneys += algorithm.getJourneys().size();
        }

        std::cout << "#### Stats ####" << std::endl;
        algorithm.getProfiler().printStatistics();
        std::cout << "Avg. journeys                : " << String::prettyDouble(numJourneys / n) << std::endl;
    }
};

class ComputeTPUsingTB : public ParameterizedCommand {
public:
    ComputeTPUsingTB(BasicShell& shell)
        : ParameterizedCommand(shell, "computeTPUsingTB", "Computs all Transfer Patterns using TB Profile Queries!")
    {
        addParameter("Input file (TripBased Data)");
        addParameter("Output file (TP Data)");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (TripBased Data)");
        const std::string outputFile = getParameter("Output file (TP Data)");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        TripBased::Data data(inputFile);
        data.printInfo();

        TransferPattern::Data tpData(data.raptorData);

        std::cout << "Computing Transfer Pattern with " << (int)numberOfThreads << " # of threads!" << std::endl;

        /* TransferPattern::TransferPatternBuilder bobTheBuilder(data); */
        /* bobTheBuilder.computeTransferPatternForStop(StopId(593)); */

        /* Graph::move(std::move(bobTheBuilder.getDAG()), tpData.transferPatternOfStop[593]); */
        /* bobTheBuilder.getDAG().printAnalysis(); */
        /* Graph::toGML(outputFile, bobTheBuilder.getDAG()); */

        if (numberOfThreads == 0) {
            TransferPattern::ComputeTransferPatternUsingTripBased(data, tpData);
        } else {
            TransferPattern::ComputeTransferPatternUsingTripBased(data, tpData, numberOfThreads, pinMultiplier);
        }

        long long totalNumVertices(0);
        long long totalNumEdges(0);

        for (const StopId stop : tpData.raptorData.stops()) {
            totalNumVertices += tpData.transferPatternOfStop[stop].numVertices();
            totalNumEdges += tpData.transferPatternOfStop[stop].numEdges();
        }

        std::cout << "Total Size:       " << String::bytesToString(tpData.byteSize()) << std::endl;
        std::cout << "Average # Nodes:  " << String::prettyDouble(totalNumVertices / data.raptorData.numberOfStops()) << std::endl;
        std::cout << "Average # Edges:  " << String::prettyDouble(totalNumEdges / data.raptorData.numberOfStops()) << std::endl;
        tpData.serialize(outputFile);
    }

private:
    inline int getNumberOfThreads() const noexcept
    {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }
};
