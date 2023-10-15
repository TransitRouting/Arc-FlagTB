#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/TransferPattern/Preprocessing/TransferPatternBuilder.h"
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

        TransferPattern::Data data(raptor);

        /* std::pair<TripId, int> result = data.directConnectionIntersection(StopId(0), StopId(5), 11 * 60 * 60); */
        /* std::cout << (int) result.first << "\t" << result.second << std::endl; */

        data.serialize(outputFile);
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
        /* tpData.serialize(outputFile); */
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
