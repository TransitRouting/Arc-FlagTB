#pragma once

#include <iostream>
#include <string>

#include "../../Algorithms/TripBased/Preprocessing/CompressARCFlags.h"
#include "../../Algorithms/TripBased/Preprocessing/ComputeARCFlagsProfile.h"
#include "../../Algorithms/TripBased/Preprocessing/StopEventGraphBuilder.h"
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

class ComputeArcFlagTB : public ParameterizedCommand {
public:
    ComputeArcFlagTB(BasicShell& shell)
        : ParameterizedCommand(shell, "computeArcFlagTB",
            "Computes Arc-Flags for the given TB Data and the "
            "given Partition File (from KaHIP).")
    {
        addParameter("Input file (TripBased Data)");
        addParameter("Output file");
        addParameter("Input file (Partition File)", "None");
        addParameter("Fixing Departure Time", "true");
        addParameter("Buffering", "true");
        addParameter("Verbose", "true");
        addParameter("Compressing", "true");
        addParameter("Number of threads", "max");
        addParameter("Pin multiplier", "1");
    }

    virtual void execute() noexcept
    {
        const std::string inputFile = getParameter("Input file (TripBased Data)");
        const std::string partitionFile = getParameter("Input file (Partition File)");
        const std::string outputFile = getParameter("Output file");
        const bool fixingDepTime = getParameter<bool>("Fixing Departure Time");
        const bool buffer = getParameter<bool>("Buffering");
        const bool verbose = getParameter<bool>("Verbose");
        const bool compress = getParameter<bool>("Compressing");
        const size_t pinMultiplier = getParameter<size_t>("Pin multiplier");

        TripBased::Data trip(inputFile);
        if (partitionFile != "None") {
            trip.raptorData.updatePartitionValuesFromFile(partitionFile, verbose);
        }
        trip.printInfo();

        if (trip.getNumberOfPartitionCells() == 1) {
            std::cout << "Number of Partition Cells is 1?\n";
            return;
        }

        TripBased::ComputeARCFlagsProfile arcFlagComputer(trip, getNumberOfThreads(), pinMultiplier);
        arcFlagComputer.computeARCFlags(fixingDepTime, buffer, verbose);

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
