/**********************************************************************************

 Copyright (c) 2023 Patrick Steil

 MIT License

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
 files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy,
 modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 is furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR
 IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

**********************************************************************************/

#pragma once

#include "../../../../DataStructures/RAPTOR/Data.h"
#include "../../../../DataStructures/RAPTOR/Entities/StopEvent.h"
#include "../../../../DataStructures/TripBased/Data.h"
#include "../../../../Helpers/Console/Progress.h"
#include "../../../../Helpers/MultiThreading.h"
#include "../../../../Helpers/String/String.h"
#include <numeric>

#include "RangeRAPTOR.h"
#include "TransitiveRAPTORModule.h"

namespace TripBased {

class ComputeARCFlagsProfileRAPTOR {

public:
    ComputeARCFlagsProfileRAPTOR(RAPTOR::Data& raptor, TripBased::Data& trip, const int numberOfThreads, const int pinMultiplier = 1)
        : raptor(raptor)
        , trip(trip)
        , numberOfThreads(numberOfThreads)
        , pinMultiplier(pinMultiplier)
    {
    }

    void computeARCFlags(const bool verbose = false)
    {
        if (verbose) {
            std::cout << "Computing ARCFlags with " << numberOfThreads << " threads." << std::endl;
        }
        Progress progress(raptor.numberOfStops());

        SimpleDynamicGraphWithARCFlag edgeListMerged;
        // SimpleEdgeListWithARCFlag edgeListMerged;
        edgeListMerged.addVertices(raptor.numberOfStopEvents());

        const int numCores = numberOfCores();
        omp_set_num_threads(numberOfThreads);
#pragma omp parallel
        {

            int threadId = omp_get_thread_num();
            pinThreadToCoreId((threadId * pinMultiplier) % numCores);
            AssertMsg(omp_get_num_threads() == numberOfThreads, "Number of threads is " << omp_get_num_threads() << ", but should be " << numberOfThreads << "!");

            RAPTOR::RangeRAPTOR::RangeRAPTOR<RAPTOR::RangeRAPTOR::TransitiveRAPTORModule<RAPTOR::NoProfiler>> bobTheRAPTORBuilder(raptor);

#pragma omp for schedule(dynamic)
            for (size_t stop = 0; stop < raptor.numberOfStops(); ++stop) {
                bobTheRAPTORBuilder.runOneToAllStops(Vertex(stop));
                ++progress;
            }

#pragma omp critical
            {
                edgeListMerged.reserve(edgeListMerged.numVertices(), edgeListMerged.numEdges() + bobTheRAPTORBuilder.getStopEventGraph().numEdges());
                for (const auto [edge, from] : bobTheRAPTORBuilder.getStopEventGraph().edgesWithFromVertex()) {
                    AssertMsg(bobTheRAPTORBuilder.getStopEventGraph().isEdge(edge), "Wrong edge??\n");
                    AssertMsg(numberOfFLAGInEdge(bobTheRAPTORBuilder.getStopEventGraph().get(ARCFlag, edge)) > 0, "there is an edge with no edge set to true!\n");
                    Vertex toVertex = bobTheRAPTORBuilder.getStopEventGraph().get(ToVertex, edge);
                    AssertMsg(edgeListMerged.isVertex(toVertex), "ToVertex is not valid!\n");

                    Edge currentEdge = edgeListMerged.findEdge(from, toVertex);
                    if (!edgeListMerged.isEdge(currentEdge)) {
                        edgeListMerged.addEdge(from, toVertex).set(ARCFlag, bobTheRAPTORBuilder.getStopEventGraph().get(ARCFlag, edge));
                        Edge newEdge = edgeListMerged.findEdge(from, toVertex);
                        AssertMsg(edgeListMerged.isEdge(newEdge) && numberOfFLAGInEdge(edgeListMerged.get(ARCFlag, newEdge)) > 0, "Something wrong after creating the edge!\n");
                    } else {
                        AssertMsg(edgeListMerged.isEdge(currentEdge), "Again, something is wrong!\n");
                        std::vector<bool>& toSetFlags = edgeListMerged.get(ARCFlag, currentEdge);
                        bitWiseOr(toSetFlags, bobTheRAPTORBuilder.getStopEventGraph().get(ARCFlag, edge));
                    }
                }
            }
        }

        if (verbose)
            std::cout << "\nFinished merging the threads, now moving to tripdata!\n";

        // now add the stopEventGraph to TB Data
        Graph::move(std::move(edgeListMerged), trip.stopEventGraph);
        trip.stopEventGraph.sortEdges(ToVertex);

        progress.finished();

        if (!verbose)
            return;

        std::cout << "Done with the preproccesing!\n";

        size_t counterOfFlags(0);

        int flagCounter(0);

        for (const auto [edge, from] : trip.stopEventGraph.edgesWithFromVertex()) {
            flagCounter = numberOfFLAGInEdge(trip.stopEventGraph.get(ARCFlag, edge));

            AssertMsg(flagCounter > 0, "There are edges with 0 flags set to true!");
            counterOfFlags += flagCounter;
        }

        std::cout << "ARC Flag Stats:\n";
        std::cout << "Number of Flags set:\t" << counterOfFlags << " (" << 100 * counterOfFlags / (trip.stopEventGraph.numEdges() * raptor.getNumberOfPartitionCells()) << "%)\n";
    }

    int numberOfFLAGInEdge(const std::vector<bool>& flags) { return std::accumulate(flags.begin(), flags.end(), 0); }

    void bitWiseOr(std::vector<bool>& to, const std::vector<bool>& from)
    {
        AssertMsg(to.size() == from.size(), "BitWiseOr - the two vectors have different length!\n");
#pragma omp simd
        for (size_t i = 0; i < to.size(); ++i) {
            to[i] = to[i] | from[i];
        }
    }

private:
    RAPTOR::Data& raptor;
    TripBased::Data& trip;
    const int numberOfThreads;
    const int pinMultiplier;
    DynamicTransferGraphWithARCFlag stopEventGraphDynamic;
};
}
