#include "../../Algorithms/TripBased/Query/ARCTransitiveQuery.h"
#include "../../Algorithms/TripBased/Query/TransitiveQuery.h"
#include "../../DataStructures/Queries/Queries.h"
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char* argv[])
{
    if (argc != 4) {
        std::cout << "Usage: ./runQueries [TB File] [Query File] [TB Original Journey File]" << std::endl;
        return -1;
    }
    TripBased::Data tripBasedData(argv[1]);
    tripBasedData.printInfo();
    TripBased::ARCTransitiveQuery<TripBased::AggregateProfiler> algorithm(tripBasedData);

    std::vector<StopQuery> queries = {};
    IO::deserialize(argv[2], queries);

    /*
    Query from 2,736 to 481 @ 15h 16m 27s
    Query from 3,408 to 3,208 @ 19h 39m 43s
    Query from 2,332 to 3,205 @ 10h 38m 34s
    Query from 3,801 to 3,210 @ 19h  2m  0s
    Query from 447 to 1,982 @ 5h  5m 41s
    Query from 2,776 to 3,663 @ 4h 13m 33s
    Query from 1,599 to 2,592 @ 21h  8m  8s
    Query from 2,483 to 2,182 @ 8h 43m 27s
    Query from 1,603 to 2,592 @ 15h 22m 37s
    Query from 75 to 2,853 @ 11h 48m 18s
    Query from 2,776 to 2,431 @ 3h 22m 32s
    Query from 33 to 2,181 @ 22h 25m 53s
    Query from 3,618 to 3,586 @ 20h 51m 17s
    Query from 1,855 to 2,293 @ 13h 30m 13s
    Query from 3,801 to 2,161 @ 10h  4m 19s
    Query from 2,922 to 2,854 @ 10h 18m 13s
    Query from 1,076 to 2,144 @ 2h 32m 19s
    Query from 3,621 to 1,485 @ 22h  5m 35s
    Query from 2,777 to 713 @ 16h  2m 15s
    Query from 3,610 to 1,798 @ 22h 15m 15
    Query form 2,776 to 2,644 @ 1922
    */

    std::vector<std::vector<RAPTOR::Journey>> originalJourneys = {};
    IO::deserialize(argv[3], originalJourneys);

    for (int i(0); i < queries.size(); ++i) {
        auto q = queries[i];
        std::cout << q << std::endl;
        algorithm.run(q.source, q.departureTime, q.target);

        std::cout << "Normal TB:" << std::endl;
        for (auto& leg : originalJourneys[i])
            std::cout << leg << std::endl;
        std::cout << "Arc-Flag TB:" << std::endl;
        for (auto& leg : algorithm.getJourneys())
            std::cout << leg << std::endl;
    }
}
