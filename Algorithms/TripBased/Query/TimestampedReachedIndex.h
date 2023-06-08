#pragma once

#include <algorithm>

#include "../../../DataStructures/TripBased/Data.h"

namespace TripBased {

class TimestampedReachedIndex {
public:
    TimestampedReachedIndex(const Data& data)
        : data(data)
        , labels(data.numberOfTrips(), -1)
        , timestamps(data.numberOfTrips(), 0)
        , timestamp(0)
        , defaultLabels(data.numberOfTrips(), -1)
    {
        for (const TripId trip : data.trips()) {
            if (data.numberOfStopsInTrip(trip) > 255)
                warning("Trip ", trip, " has ", data.numberOfStopsInTrip(trip), " stops!");
            defaultLabels[trip] = data.numberOfStopsInTrip(trip);
        }
    }

public:
    inline void clear() noexcept
    {
        ++timestamp;
        if (timestamp == 0) {
            labels = defaultLabels;
            std::fill(timestamps.begin(), timestamps.end(), 0);
        }
    }

    inline StopIndex operator()(const TripId trip) noexcept
    {
        AssertMsg(trip < labels.size(), "Trip " << trip << " is out of bounds!");
        return StopIndex(getLabel(trip));
    }

    inline bool alreadyReached(const TripId trip, const u_int8_t index) noexcept
    {
        return getLabel(trip) <= index;
    }

    inline void update(const TripId trip, const StopIndex index) noexcept
    {
        AssertMsg(trip < labels.size(), "Trip " << trip << " is out of bounds!");
        const TripId routeEnd = data.firstTripOfRoute[data.routeOfTrip[trip] + 1];
        for (TripId i = trip; i < routeEnd; i++) {
            u_int8_t& label = getLabel(i);
            if (label <= index)
                break;
            label = index;
        }
    }

private:
    inline u_int8_t& getLabel(const TripId trip) noexcept
    {
        if (timestamps[trip] != timestamp) {
            labels[trip] = defaultLabels[trip];
            timestamps[trip] = timestamp;
        }
        return labels[trip];
    }

    const Data& data;

    std::vector<u_int8_t> labels;
    std::vector<u_int16_t> timestamps;
    u_int16_t timestamp;

    std::vector<u_int8_t> defaultLabels;
};

} // namespace TripBased
