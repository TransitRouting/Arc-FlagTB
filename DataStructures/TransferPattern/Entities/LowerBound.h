#pragma once

namespace TransferPattern {

struct LowerBound {
    LowerBound(std::vector<int> travelTimes = {}, std::vector<uint8_t> numberOfTrips = {})
        : travelTimes(travelTimes)
        , numberOfTrips(numberOfTrips)
    {
    }

    inline void resize(const size_t size) {
        travelTimes.assign(size, 0);
        numberOfTrips.assign(size, 0);
    }

    inline void setTravelTimes(std::vector<int>& newTravelTimes) {
        travelTimes = newTravelTimes;
    }

    inline void setNumberOfTrips(std::vector<uint8_t>& newNumberOfTrips) {
        numberOfTrips = newNumberOfTrips;
    }

    inline void serialize(IO::Serialization& serialize) const
    {
        serialize(travelTimes, numberOfTrips);
    }

    inline void deserialize(IO::Deserialization& deserialize)
    {
        deserialize(travelTimes, numberOfTrips);
    }

    std::vector<int> travelTimes;
    std::vector<uint8_t> numberOfTrips;
};

}
