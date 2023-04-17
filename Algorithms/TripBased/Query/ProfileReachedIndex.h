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

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class ProfileReachedIndex {
public:
    ProfileReachedIndex(const Data& data)
        : data(data)
        , labels(data.numberOfTrips() << 4, -1)
        , defaultLabels(data.numberOfTrips() << 4, -1)
    {
        for (const TripId trip : data.trips()) {
            if (data.numberOfStopsInTrip(trip) > 255)
                warning("Trip ", trip, " has ", data.numberOfStopsInTrip(trip), " stops!");
            u_int8_t numberOfStops = data.numberOfStopsInTrip(trip);
#pragma omp simd
            for (int i = 0; i < 16; ++i)
                defaultLabels[(trip << 4) + i] = numberOfStops;
        }
    }

public:
    inline void clear() noexcept
    {
        labels = defaultLabels;
    }

    inline void clear(const RouteId route) noexcept
    {
        const TripId start = data.firstTripOfRoute[route];
        const TripId end = data.firstTripOfRoute[route + 1];
#pragma omp simd
        for (size_t i = (start << 4); i < (end << 4); ++i)
            labels[i] = defaultLabels[i];
    }

    inline StopIndex operator()(const TripId trip, const int n = 1) const noexcept
    {
        AssertMsg((trip << 4) < labels.size(), "Trip " << trip << " is out of bounds!");
        AssertMsg(n <= 16, "This number of transfers is not supported");
        return StopIndex(labels[(trip << 4) + n - 1]);
    }

    inline bool alreadyReached(const TripId trip, const u_int8_t index, const int n = 1) const noexcept
    {
        AssertMsg(n <= 16, "This number of transfers is not supported");
        return labels[(trip << 4) + n - 1] <= index;
    }

    inline void update(const TripId trip, const StopIndex index, const int n = 1) noexcept
    {
        AssertMsg((trip << 4) < labels.size(), "Trip " << trip << " is out of bounds!");
        AssertMsg(n <= 16, "This number of transfers is not supported");
        const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
        for (TripId tripIndex = trip; tripIndex < routeEnd; tripIndex++) {
            for (int i = n - 1; i < 16; ++i) {
                if (labels[(tripIndex << 4) + i] <= index)
                    break;
                labels[(tripIndex << 4) + i] = index;
            }
        }
    }

    inline void updateRaw(const TripId trip, const TripId tripEnd, const StopIndex index, const int n = 1) noexcept
    {
        AssertMsg((trip << 4) < labels.size(), "Trip " << trip << " is out of bounds!");
        AssertMsg(n <= 16, "This number of transfers is not supported");
        AssertMsg(tripEnd <= data.firstTripOfRoute[data.routeOfTrip[trip] + 1],
            "Trip end" << tripEnd << " is out of bounds!");
        for (TripId tripIndex(trip); tripIndex < tripEnd; ++tripIndex) {
#pragma omp simd
            for (int i = n - 1; i < 16; ++i) {
                labels[(tripIndex << 4) + i] = index;
            }
        }
    }

private:
    const Data& data;

    std::vector<u_int8_t> labels;

    std::vector<u_int8_t> defaultLabels;
};

} // namespace TripBased
