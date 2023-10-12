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
        /* addParameter("Output file"); */
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (TripBased Data)");
        /* const std::string outputFile = getParameter("Output file"); */
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        TripBased::Data data(inputFile);
        data.printInfo();

        if (numberOfThreads == 0) {
            TransferPattern::ComputeTransferPatternUsingTripBased(data);
        } else {
            TransferPattern::ComputeTransferPatternUsingTripBased(data, numberOfThreads, pinMultiplier);
        }

        /* data.serialize(outputFile); */
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
