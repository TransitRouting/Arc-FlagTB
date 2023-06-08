#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/TripBased/Preprocessing/ARCFlagTBBuilder.h"
/* #include "../../Algorithms/TripBased/Preprocessing/ComputeARCFlagsProfile.h" */
#include "../../Algorithms/TripBased/Preprocessing/CompressARCFlags.h"
#include "../../Algorithms/TripBased/Preprocessing/RangeRAPTOR/ComputeARCFlagsProfileRAPTOR.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
#include "../../Algorithms/TripBased/Preprocessing/ULTRABuilderTransitive.h"
#include "../../DataStructures/Graph/Graph.h"
#include "../../DataStructures/RAPTOR/Data.h"
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
        addParameter("Input file (Partition File)", "None");
        addParameter("Route-based pruning?", "true");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (RAPTOR Data)");
        const std::string partitionFile = getParameter("Input file (Partition File)");
        const std::string outputFile = getParameter("Output file");
        const bool routeBasedPruning = getParameter<bool>("Route-based pruning?");
        const int numberOfThreads = getNumberOfThreads();
        const int pinMultiplier = getParameter<int>("Pin multiplier");

        RAPTOR::Data raptor(inputFile);
        if (partitionFile != "None") {
            raptor.updatePartitionValuesFromFile(partitionFile, true);
            raptor.serialize(inputFile);
        }
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

class ComputeArcFlagTB : public ParameterizedCommand {
public:
    ComputeArcFlagTB(BasicShell& shell)
        : ParameterizedCommand(shell, "computeArcFlagTB",
            "Computes Arc-Flags for the given TB Data!")
    {
        addParameter("Input file (TripBased Data)");
        addParameter("Output file");
        addParameter("Verbose", "true");
        addParameter("Compressing", "true");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (TripBased Data)");
        const std::string outputFile = getParameter("Output file");
        const bool verbose = getParameter<bool>("Verbose");
        const bool compress = getParameter<bool>("Compressing");
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");

        TripBased::Data trip(inputFile);
        trip.printInfo();

        if (trip.getNumberOfPartitionCells() == 1) {
            std::cout << "Number of Partition Cells is 1?\n";
            return;
        }

        TripBased::ARCFlagTBBuilder arcFlagComputer(trip, getNumberOfThreads(), pinMultiplier);
        arcFlagComputer.computeARCFlags(verbose);
        /* TripBased::ComputeARCFlagsProfile arcFlagComputer(trip, getNumberOfThreads(), pinMultiplier); */
        /* arcFlagComputer.computeARCFlags(true, true, verbose); */

        trip.serialize(outputFile);

        if (compress) {
            TripBased::CompressARCFlags(inputFile);
        }
    }

private:
    inline size_t getNumberOfThreads() const noexcept
    {
        if (getParameter("Number of threads") == "max") {
            return numberOfCores();
        } else {
            return getParameter<int>("Number of threads");
        }
    }
};

class ApplyPartitionToTripBased : public ParameterizedCommand {
public:
    ApplyPartitionToTripBased(BasicShell& shell)
        : ParameterizedCommand(shell, "applyPartitionToTripBased", "Applies the given partition to the stops of the Trip-Based input and saves it.")
    {
        addParameter("Input file (Trip Data)");
        addParameter("Input file (Partition File)");
        addParameter("Output file (Trip Data)");
        addParameter("Verbose", "true");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (Trip Data)");
        const std::string outputFile = getParameter("Output file (Trip Data)");
        const std::string partitionFile = getParameter("Input file (Partition File)");
        const bool verbose = getParameter<bool>("Verbose");

        TripBased::Data trip(inputFile);
        trip.printInfo();
        trip.updatePartitionValuesFromFile(partitionFile, verbose);
        trip.printInfo();
        trip.serialize(outputFile);
    }
};

class ComputeArcFlagTBRAPTOR : public ParameterizedCommand {
public:
    ComputeArcFlagTBRAPTOR(BasicShell& shell)
        : ParameterizedCommand(shell, "computeArcFlagTBRAPTOR",
            "Computes Arc-Flags for the given RAPTOR Data and the "
            "given Partition File (from KaHIP) using the RangeRAPTOR with ULTRA conditions.")
    {
        addParameter("Input file (RAPTOR Data)");
        addParameter("Output file (Trip Data)");
        addParameter("Input file (Partition File)", "None");
        addParameter("Verbose", "true");
        addParameter("Compressing", "true");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (RAPTOR Data)");
        const std::string partitionFile = getParameter("Input file (Partition File)");
        const std::string outputFile = getParameter("Output file (Trip Data)");
        const bool verbose = getParameter<bool>("Verbose");
        const bool compress = getParameter<bool>("Compressing");
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");

        RAPTOR::Data raptor(inputFile);
        if (partitionFile != "None") {
            raptor.updatePartitionValuesFromFile(partitionFile, verbose);
        }
        raptor.useImplicitDepartureBufferTimes();
        raptor.useImplicitArrivalBufferTimes();
        raptor.printInfo();

        if (raptor.getNumberOfPartitionCells() == 1) {
            std::cout << "Number of Partition Cells is 1?\n";
            return;
        }

        TripBased::Data trip(raptor);

        TripBased::ComputeARCFlagsProfileRAPTOR arcFlagComputer(raptor, trip, getNumberOfThreads(), pinMultiplier);
        arcFlagComputer.computeARCFlags(verbose);

        trip.serialize(outputFile);

        if (compress) {
            TripBased::CompressARCFlags(outputFile);
        }
    }

private:
    inline size_t getNumberOfThreads() const noexcept
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

class ComputePageRank : public ParameterizedCommand {
public:
    ComputePageRank(BasicShell& shell)
        : ParameterizedCommand(shell, "computePageRank",
            "Computes the pagerank values for the given layout graph.")
    {
        addParameter("Input file");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file");

        DynamicGraphWithWeights originalGraph(inputFile);
        DynamicGraphWithWeights graph;
        Graph::copy(originalGraph, graph);
        graph.revert();

        std::vector<double> pr = Graph::pagerank(originalGraph, graph);
        for (size_t node(0); node < pr.size(); ++node)
            std::cout << (int)node << "," << pr[node] << "\n";
        std::cout << std::endl;
    }
};
