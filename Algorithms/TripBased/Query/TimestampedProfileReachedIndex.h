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

class TimestampedProfileReachedIndex {
public:
    TimestampedProfileReachedIndex(const Data& data)
        : data(data)
        , labels(data.numberOfTrips() << 4, -1)
	, timestamps(data.numberOfTrips() << 4, 0)
        , timestamp(0)
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
	if (timestamp == 0) {
	     labels = defaultLabels;
	     std::fill(timestamps.begin(), timestamps.end(), 0);
	}
	++timestamp;
    }


    inline void clear(const RouteId route) noexcept
    {
        const TripId start = data.firstTripOfRoute[route];
        const TripId end = data.firstTripOfRoute[route + 1];
#pragma omp simd
	for (size_t i = (start << 4); i < (end << 4); ++i) {
            labels[i] = defaultLabels[i];
	    timestamps[i] = timestamp;
	}
    }

    inline StopIndex operator()(const TripId trip, const int n = 1) noexcept
    {
        AssertMsg((trip << 4) < labels.size(), "Trip " << trip << " is out of bounds!");
        AssertMsg(n <= 16, "This number of transfers is not supported");
        return StopIndex(getLabel(trip, n));
    }

    inline bool alreadyReached(const TripId trip, const u_int8_t index, const int n = 1) noexcept
    {
        AssertMsg(n <= 16, "This number of transfers is not supported");
        return getLabel(trip, n) <= index;
    }

    inline void update(const TripId trip, const StopIndex index, const int n = 1) noexcept
    {
        AssertMsg((trip << 4) < labels.size(), "Trip " << trip << " is out of bounds!");
        AssertMsg(n <= 16, "This number of transfers is not supported");
        const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
        for (TripId tripIndex = trip; tripIndex < routeEnd; ++tripIndex) {
            for (int i = n - 1; i < 16; ++i) {
		u_int8_t& label = getLabel(tripIndex, i + 1);
                if (label <= index)
                    break;
		label = index;
            }
        }
    }

private:
    inline u_int8_t& getLabel(const TripId trip, const int n = 1) noexcept
    {
        if (timestamps[(trip << 4) + n - 1] != timestamp) {
	    labels[(trip << 4) + n - 1] = defaultLabels[(trip << 4) + n - 1];
            timestamps[(trip << 4) + n - 1] = timestamp;
        }
        return labels[(trip << 4) + n - 1];
    }
    const Data& data;

    std::vector<u_int8_t> labels;
    std::vector<u_int16_t> timestamps;
    u_int16_t timestamp;

    std::vector<u_int8_t> defaultLabels;
};

} // namespace TripBased
