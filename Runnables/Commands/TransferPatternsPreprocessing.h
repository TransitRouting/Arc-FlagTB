#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/TransferPattern/Preprocessing/TransferPatternBuilder.h"
#include "../../Algorithms/TransferPattern/Query/Query.h"

#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TransferPattern/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class RAPTORToTransferPattern : public ParameterizedCommand {
public:
    RAPTORToTransferPattern(BasicShell& shell)
        : ParameterizedCommand(shell, "raptorToTransferPattern", "Converts RAPTOR to TP")
    {
        addParameter("Input file (RAPTOR Data)");
        addParameter("Output file (TP Data)");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (RAPTOR Data)");
        const std::string outputFile = getParameter("Output file (TP Data)");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();

        TransferPattern::Data input(raptor);
        input.serialize(outputFile);

        TransferPattern::Data data(outputFile);

        data.printInfo();
        /* TransferPattern::Query<TransferPattern::AggregateProfiler> query(data); */

        /* query.run(StopId(593), 10 * 60 * 60, StopId(10)); */

        /* query.getProfiler().printStatistics(); */

        /* data.serialize(outputFile); */
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
