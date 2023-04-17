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
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "../../../DataStructures/Graph/Graph.h"
#include "../../../DataStructures/TripBased/Data.h"
#include "../../../Helpers/Console/Progress.h"
#include "../../../Helpers/IO/Serialization.h"

namespace TripBased {

bool sortbysecDESC(const std::pair<std::vector<bool>, int>& a, const std::pair<std::vector<bool>, int>& b)
{
    return (a.second > b.second);
}

inline void CompressARCFlags(const std::string tripFileName = "", const std::string seperator = ".")
{
    std::vector<std::vector<bool>> arcflags;
    IO::deserialize(tripFileName + seperator + "graph" + seperator + "aRCFlag", arcflags);

    std::unordered_map<std::vector<bool>, size_t> map;

    Progress progress(arcflags.size() << 1);

    for (size_t i(0); i < arcflags.size(); ++i) {
        if (map[arcflags[i]])
            map[arcflags[i]]++;
        else
            map[arcflags[i]] = 1;
        progress++;
    }

    std::vector<std::pair<std::vector<bool>, int>> allFlagsAsPair;
    allFlagsAsPair.reserve(map.bucket_count());

    std::vector<std::vector<bool>> flags;
    flags.reserve(allFlagsAsPair.size());

    for (auto it = map.begin(); it != map.end(); ++it)
        allFlagsAsPair.push_back(std::make_pair(it->first, it->second));

    std::sort(allFlagsAsPair.begin(), allFlagsAsPair.end(), sortbysecDESC);

    for (unsigned long int i(0); i < allFlagsAsPair.size(); ++i) {
        map[allFlagsAsPair[i].first] = i;
        flags.push_back(allFlagsAsPair[i].first);
    }

    std::vector<unsigned long int> indizes;
    indizes.reserve(arcflags.size());

    for (const std::vector<bool>& flag : arcflags) {
        indizes.push_back((unsigned long int)map[flag]);
        progress++;
    }

    progress.finished();

    std::cout << "Saving the compressed flags!\n";
    IO::serialize(tripFileName + seperator + "graph" + seperator + "index", indizes);
    IO::serialize(tripFileName + seperator + "graph" + seperator + "flagscompressed", flags);
    std::cout << "Done with compressed flags!\n";
}
} // namespace TripBased
