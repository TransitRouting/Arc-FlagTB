#pragma once

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"
#include "../../../DataStructures/RAPTOR/Data.h"
#include "../../../Helpers/MultiThreading.h"
#include "../../../Helpers/Timer.h"
#include "../../../Helpers/Console/Progress.h"

#include "ShortcutSearchTransitive.h"

namespace TripBased {

template<bool DEBUG = false>
class ULTRABuilderTransitive {

public:
    inline static constexpr bool Debug = DEBUG;
    using Type = ULTRABuilderTransitive<Debug>;
    using ShortcutSearchType = ShortcutSearchTransitive<Debug>;

public:
    ULTRABuilderTransitive(const Data& data) :
        data(data) {
        stopEventGraph.addVertices(data.numberOfStopEvents());
    }

    void computeShortcuts(const ThreadPinning& threadPinning, const int minDepartureTime = -never, const int maxDepartureTime = never, const bool verbose = true) noexcept {
        if (verbose) std::cout << "Computing shortcuts with " << threadPinning.numberOfThreads << " threads." << std::endl;

        std::vector<Shortcut> shortcuts;

        Progress progress(data.numberOfStops(), verbose);
        omp_set_num_threads(threadPinning.numberOfThreads);
        #pragma omp parallel
        {
            threadPinning.pinThread();

            ShortcutSearchType shortcutSearch(data);

            #pragma omp for schedule(dynamic)
            for (size_t i = 0; i < data.numberOfStops(); i++) {
                shortcutSearch.run(StopId(i), minDepartureTime, maxDepartureTime);
                progress++;
            }

            #pragma omp critical
            {
                const std::vector<Shortcut>& localShortcuts = shortcutSearch.getShortcuts();
                for (const Shortcut& shortcut : localShortcuts) {
                    shortcuts.emplace_back(shortcut);
                }
            }
        }

        if (!shortcuts.empty()) {
            std::sort(shortcuts.begin(), shortcuts.end(), [](const Shortcut& a, const Shortcut& b){
                return (a.origin < b.origin) || ((a.origin == b.origin) && (a.destination < b.destination));
            });
	    std::vector<bool> emptyFlags(data.getNumberOfPartitionCells(), false);
            stopEventGraph.addEdge(Vertex(shortcuts[0].origin), Vertex(shortcuts[0].destination)).set(TravelTime, shortcuts[0].walkingDistance).set(ARCFlag, emptyFlags);
            for (size_t i = 1; i < shortcuts.size(); i++) {
                if ((shortcuts[i].origin == shortcuts[i - 1].origin) && (shortcuts[i].destination == shortcuts[i - 1].destination)) continue;
		std::vector<bool> emptyFlags(data.getNumberOfPartitionCells(), false);
                stopEventGraph.addEdge(Vertex(shortcuts[i].origin), Vertex(shortcuts[i].destination)).set(TravelTime, shortcuts[i].walkingDistance).set(ARCFlag, emptyFlags);
            }
            stopEventGraph.sortEdges(ToVertex);
        }

        progress.finished();
    }

    inline const DynamicTransferGraphWithARCFlag& getStopEventGraph() const noexcept {
        return stopEventGraph;
    }

    inline DynamicTransferGraphWithARCFlag& getStopEventGraph() noexcept {
        return stopEventGraph;
    }

private:
    const Data& data;
    DynamicTransferGraphWithARCFlag stopEventGraph;

};

}
