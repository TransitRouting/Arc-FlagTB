#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/ULTRABuilderTransitive.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
#include "../../DataStructures/TransferPattern/Data.h"
#include "../../DataStructures/TripBased/Data.h"
#include "../../Helpers/MultiThreading.h"
#include "../../Helpers/String/String.h"
#include "../../Shell/Shell.h"

using namespace Shell;

class RAPTORToTripBased : public ParameterizedCommand {
public:
    RAPTORToTripBased(BasicShell& shell)
        : ParameterizedCommand(shell, "raptorToTripBased",
            "Converts stop-to-stop transfers to event-to-event transfers and "
            "saves the resulting network in Trip-Based format.")
    {
        addParameter("Input file (RAPTOR Data)");
        addParameter("Output file");
        addParameter("Route-based pruning?", "true");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (RAPTOR Data)");
        const std::string outputFile = getParameter("Output file");
        const bool routeBasedPruning = getParameter<bool>("Route-based pruning?");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        if (numberOfThreads == 0) {
            if (routeBasedPruning) {
                TripBased::ComputeStopEventGraphRouteBased(data);
            } else {
                TripBased::ComputeStopEventGraph(data);
            }
        } else {
            if (routeBasedPruning) {
                TripBased::ComputeStopEventGraphRouteBased(data, numberOfThreads, pinMultiplier);
            } else {
                TripBased::ComputeStopEventGraph(data, numberOfThreads, pinMultiplier);
            }
        }

        data.printInfo();
        data.serialize(outputFile);
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

        std::pair<TripId, int> result = data.directConnectionIntersection(StopId(0), StopId(5), 11 * 60 * 60);
        std::cout << (int) result.first << "\t" << result.second << std::endl;

        data.serialize(outputFile);
    }
};

class ComputeTransitiveEventToEventShortcuts : public ParameterizedCommand {

public:
    ComputeTransitiveEventToEventShortcuts(BasicShell& shell)
        : ParameterizedCommand(shell, "computeTransitiveEventToEventShortcuts", "Computes transitive event-to-event transfer shortcuts using ULTRA and saves the resulting network in Trip-Based format.")
    {
        addParameter("Input file");
        addParameter("Output file");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();
        TripBased::Data data(raptor);

        TripBased::ULTRABuilderTransitive<false> shortcutGraphBuilder(data);
        std::cout << "Computing transitive event-to-event ULTRA shortcuts (parallel with " << numberOfThreads << " threads)." << std::endl;
        shortcutGraphBuilder.computeShortcuts(ThreadPinning(numberOfThreads, pinMultiplier));
        Graph::move(std::move(shortcutGraphBuilder.getStopEventGraph()), data.stopEventGraph);

        data.printInfo();
        data.serialize(outputFile);
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

class CreateLayoutGraph : public ParameterizedCommand {
public:
    CreateLayoutGraph(BasicShell& shell)
        : ParameterizedCommand(shell, "createLayoutGraph",
            "Reads RAPTOR and creates the Layout Graph and saves it in the "
            "METIS graph format (readable by KaHIP).")
    {
        addParameter("Input file");
        addParameter("Output file");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file");
        const std::string outputFile = getParameter("Output file");

        RAPTOR::Data raptor(inputFile);
        raptor.printInfo();

        raptor.createGraphForMETIS(RAPTOR::TRIP_WEIGHTED | RAPTOR::TRANSFER_WEIGHTED, true);
        raptor.writeMETISFile(outputFile, true);

        Graph::toGML(outputFile, raptor.layoutGraph);
    }
};
