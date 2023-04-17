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

#include "../../../../DataStructures/Container/Set.h"
#include "../../../../DataStructures/RAPTOR/Entities/ArrivalLabel.h"
#include "../../../../DataStructures/RAPTOR/Entities/Journey.h"
#include "../../../../DataStructures/RAPTOR/Entities/RouteSegment.h"
#include "../../../../DataStructures/TripBased/Data.h"
#include "../../../../Helpers/String/String.h"
#include <unordered_map>

#include "../../Query/ProfileReachedIndex.h"

#include "RangeRAPTOR.h"
#include "TransitiveRAPTORModule.h"

namespace TripBased {

class CalculateARCFlagsProfile {

public:
    CalculateARCFlagsProfile(RAPTOR::Data& data)
        : data(data)
        , numberOfPartitions(data.raptorData.numberOfPartitions)
    {
    }

    inline void run(const Vertex source) noexcept
    {
        rangeRAPTOR.runOneToAllStops(source);
    }

private:
    RAPTOR::Data& data;
    RAPTOR::RangeRAPTOR::RangeRAPTOR rangeRAPTOR;

    int numberOfPartitions;

    SimpleDynamicGraphWithARCFlag stopEventGraphOfThread;
};

}
